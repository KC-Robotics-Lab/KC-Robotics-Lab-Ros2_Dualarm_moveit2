#!/usr/bin/python3

import threading
import time
import struct
import zmq
import queue

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

from std_srvs.srv import Empty
from dual_arm_msg.srv import Rotblock
from dual_arm_msg.srv import Objpickpoint


# ===== 공통 유틸: Future 동기 대기 =====
def wait_future_blocking(node: Node, fut: Future, timeout: float) -> bool:
    end_t = time.time() + timeout
    while rclpy.ok() and (time.time() < end_t):
        if fut.done():
            return True
        time.sleep(0.01)  # 10ms 폴링
    return fut.done()


# ===== 서비스 호출 유틸 =====
def call_empty(node: Node, client, timeout=3.0):
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f"[SRV] {client.srv_name} not available")
        return False

    fut: Future = client.call_async(Empty.Request())
    ok = wait_future_blocking(node, fut, timeout)
    if not ok or fut.result() is None:
        node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
        return False

    node.get_logger().info(f"[SRV] {client.srv_name} → OK")
    return True


def call_angles(node: Node, client, rx: float, ry: float, rz: float, timeout=3.0):
    def normalize_deg(a: float) -> float:
        a = float(a)
        if a > 180.0:
            a -= 360.0
        return a

    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f"[SRV] {client.srv_name} not available")
        return False

    req = Rotblock.Request()
    req.r_rx_deg = normalize_deg(rx)
    req.r_ry_deg = normalize_deg(ry)
    req.r_rz_deg = normalize_deg(rz)

    fut: Future = client.call_async(req)
    ok = wait_future_blocking(node, fut, timeout)
    if not ok or fut.result() is None:
        node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
        return False

    resp = fut.result()
    node.get_logger().info(
        f"[SRV] {client.srv_name} → success={resp.success}, msg='{resp.message}'"
    )
    return bool(resp.success)


def call_objpickpoint(node: Node, client, right: tuple, left: tuple, timeout: float = 3.0):
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f"[SRV] {client.srv_name} not available")
        return False

    req = Objpickpoint.Request()
    req.right_x, req.right_y, req.right_z, req.right_rx, req.right_ry, req.right_rz = map(float, right)
    req.left_x,  req.left_y,  req.left_z,  req.left_rx,  req.left_ry,  req.left_rz  = map(float, left)

    fut: Future = client.call_async(req)
    ok = wait_future_blocking(node, fut, timeout)
    if not ok or fut.result() is None:
        node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
        return False

    resp = fut.result()
    node.get_logger().info(
        f"[SRV] {client.srv_name} → success={resp.success}, msg='{resp.message}'"
    )
    return bool(resp.success)


class SequenceNode(Node):
    # ===== 하드코딩 설정 =====
    SRV_START_MAINSEQ  = "start_mainseq"
    SRV_STEP4_OK       = "/cam_check_prev"
    SRV_STEP5_OK       = "cam_check"
    SRV_FINISH_MAINSEQ = "finish_mainseq"
    SRV_ANGLE_CMD      = "angle_cmd"
    SRV_OBJPICKPOINT   = "objpickpoint"

    ADDR_CALC_CMD_REQ      = "tcp://127.0.0.1:7008"
    ADDR_ANGLE_GET_REQ     = "tcp://127.0.0.1:7009"
    ADDR_PICKPOINT_GET_REQ = "tcp://127.0.0.1:7011"

    POLL_ANGLE_MS     = 100
    POLL_PICKPOINT_MS = 100

    def __init__(self):
        super().__init__("sequence_node")

        # ---- ROS Clients/Servers ----
        self.cli_step_start   = self.create_client(Empty,    self.SRV_START_MAINSEQ)
        self.cli_angle        = self.create_client(Rotblock, self.SRV_ANGLE_CMD)
        self.cli_objpickpoint = self.create_client(Objpickpoint, self.SRV_OBJPICKPOINT)

        self.create_service(Empty, self.SRV_STEP4_OK,       self._cb_step4_ok)
        self.create_service(Empty, self.SRV_STEP5_OK,       self._cb_step5_ok)
        self.create_service(Empty, self.SRV_FINISH_MAINSEQ, self._cb_step_done)

        # ---- ZMQ Context ----
        self._ctx = zmq.Context.instance()

        # ---- 상태 ----
        self._seq_active = False
        self._node_running = True
        self._seq_completed = False
        self._start_after_pickpoint = False
        self._sent_pickpoint = False

        # cam_check 콜백 -> 큐로 넘김 (콜백에서 소켓 사용 금지)
        self._cmd_q: "queue.Queue[bytes]" = queue.Queue()

        # 시작 시 stale flush (임시 소켓)
        self._flush_stale_angles(max_flush=5)
        self._flush_stale_pickpoints(max_flush=5)

        # 스레드 시작
        threading.Thread(target=self._calc_cmd_worker, daemon=True).start()
        threading.Thread(target=self.get_angle_loop, daemon=True).start()
        threading.Thread(target=self.get_pickpoint_loop, daemon=True).start()

        self.get_logger().info("SequenceNode ready. (no keyboard, no block_out)")

    # ---------- ZMQ helpers ----------
    def _make_req_socket_with_monitor(self, addr: str, name: str):
        s = self._ctx.socket(zmq.REQ)
        s.setsockopt(zmq.LINGER, 0)

        # REQ 타임아웃/재시도 시 상태 꼬임 완화
        s.setsockopt(zmq.REQ_RELAXED, 1)
        s.setsockopt(zmq.REQ_CORRELATE, 1)

        s.setsockopt(zmq.SNDTIMEO, 2000)
        s.setsockopt(zmq.RCVTIMEO, 2000)

        s.connect(addr)

        mon_addr = f"inproc://mon-{name}-{id(s)}"
        s.monitor(mon_addr, zmq.EVENT_CONNECTED | zmq.EVENT_CONNECT_DELAYED | zmq.EVENT_DISCONNECTED)
        mon = self._ctx.socket(zmq.PAIR)
        mon.connect(mon_addr)

        self.get_logger().info(f"[ZMQ:{name}] connect -> {addr}")
        return s, mon

    def _close_req_socket_with_monitor(self, s, mon, name: str):
        try:
            mon.close(0)
        except Exception:
            pass
        try:
            s.disable_monitor()
        except Exception:
            pass
        try:
            s.close(0)
        except Exception:
            pass
        self.get_logger().info(f"[ZMQ:{name}] socket closed")

    def _drain_monitor(self, mon, name: str):
        # 이벤트가 있을 때만 읽음 (스팸 방지)
        try:
            while mon.poll(0, zmq.POLLIN):
                evt = mon.recv_multipart()
                self.get_logger().info(f"[ZMQ:{name}] MON evt frames={len(evt)}")
        except Exception as e:
            self.get_logger().warning(f"[ZMQ:{name}] MON read error: {e}")

    def _req_roundtrip(self, s, mon, name: str, frames: list[bytes], poll_ms: int = 2000):
        self._drain_monitor(mon, name)

        try:
            self.get_logger().debug(f"[ZMQ:{name}] SEND {frames}")
            s.send_multipart(frames)
        except Exception as e:
            self.get_logger().error(f"[ZMQ:{name}] SEND failed: {e}")
            return None, "send_fail"

        try:
            if s.poll(poll_ms, zmq.POLLIN) == 0:
                self.get_logger().warning(f"[ZMQ:{name}] RECV timeout (no reply in {poll_ms}ms)")
                return None, "timeout"
            rep = s.recv_multipart()
            self.get_logger().debug(f"[ZMQ:{name}] RECV {rep}")
            return rep, "ok"
        except Exception as e:
            self.get_logger().error(f"[ZMQ:{name}] RECV failed: {e}")
            return None, "recv_fail"

    # ---------- Flush stale (임시 소켓) ----------
    def _flush_stale_angles(self, max_flush: int = 5):
        self.get_logger().info(f"[ZMQ] Flushing stale ANGLE_GET values (max={max_flush})...")
        s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE_FLUSH")
        try:
            for i in range(max_flush):
                rep, st = self._req_roundtrip(s, mon, "ANGLE_FLUSH", [b"ANGLE_GET?"], poll_ms=800)
                if st != "ok" or (not rep) or rep[0] != b"OK":
                    self.get_logger().info(f"[ZMQ] angle flush done at iter={i}, st={st}, rep={rep}")
                    break
                self.get_logger().warning(f"[ZMQ] flushed stale angle #{i}: frames={len(rep)}")
        finally:
            self._close_req_socket_with_monitor(s, mon, "ANGLE_FLUSH")
        self.get_logger().info("[ZMQ] ANGLE flush finished.")

    def _flush_stale_pickpoints(self, max_flush: int = 5):
        self.get_logger().info(f"[ZMQ] Flushing stale PICK_POINT_GET values (max={max_flush})...")
        s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK_FLUSH")
        try:
            for i in range(max_flush):
                rep, st = self._req_roundtrip(s, mon, "PICK_FLUSH", [b"PICK_POINT_GET?"], poll_ms=800)
                if st != "ok" or (not rep) or rep[0] != b"OK":
                    self.get_logger().info(f"[ZMQ] pick flush done at iter={i}, st={st}, rep={rep}")
                    break
                self.get_logger().warning(f"[ZMQ] flushed stale pickpoint #{i}: frames={len(rep)}")
        finally:
            self._close_req_socket_with_monitor(s, mon, "PICK_FLUSH")
        self.get_logger().info("[ZMQ] PICKPOINT flush finished.")

    # ---------- ROS callbacks ----------
    def _cb_step4_ok(self, req, resp):
        if self._seq_active:
            self.get_logger().info("[RECV] cam_check_prev → angle_cmd(0,0,0)")
            ok = call_angles(self, self.cli_angle, 0.0, 0.0, 0.0)
            if not ok:
                self.get_logger().error("[SRV] angle_cmd(0,0,0) failed")
        return resp

    def _cb_step5_ok(self, req, resp):
        if self._seq_active:
            self.get_logger().info("[RECV] cam_check → enqueue CALC_ANGLE_CMD(True)")
            try:
                self._cmd_q.put_nowait(b"\x01")
            except Exception as e:
                self.get_logger().error(f"[ZMQ] enqueue failed: {e}")
        return resp
    
    def _cb_step_done(self, req, resp):
        if self._seq_active:
            self.get_logger().info("[RECV] finish_mainseq → stop (no restart)")
            self._seq_active = False
            self._sent_pickpoint = False
            self._seq_completed = True
        return resp

    # ---------- ZMQ workers ----------
    def _calc_cmd_worker(self):
        self.get_logger().info("[ZMQ] calc_cmd_worker started")
        s, mon = self._make_req_socket_with_monitor(self.ADDR_CALC_CMD_REQ, "CMD")
        try:
            while self._node_running:
                try:
                    flag = self._cmd_q.get(timeout=0.2)  # b"\x01" / b"\x00"
                except queue.Empty:
                    continue

                rep, st = self._req_roundtrip(s, mon, "CMD", [b"CALC_ANGLE_CMD", flag[:1]], poll_ms=2000)
                if st == "timeout":
                    # timeout이면 소켓 꼬임 방지: 재생성
                    self.get_logger().warning("[ZMQ:CMD] timeout -> recreate socket")
                    self._close_req_socket_with_monitor(s, mon, "CMD")
                    s, mon = self._make_req_socket_with_monitor(self.ADDR_CALC_CMD_REQ, "CMD")
        finally:
            self._close_req_socket_with_monitor(s, mon, "CMD")
            self.get_logger().info("[ZMQ] calc_cmd_worker exited")

    def get_angle_loop(self):
        self.get_logger().info("[ZMQ] get_angle_loop started")
        s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE")
        try:
            while self._node_running:
                rep, st = self._req_roundtrip(s, mon, "ANGLE", [b"ANGLE_GET?"], poll_ms=2000)

                if st == "timeout":
                    self.get_logger().warning("[ZMQ:ANGLE] timeout -> recreate socket")
                    self._close_req_socket_with_monitor(s, mon, "ANGLE")
                    s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE")
                    time.sleep(0.1)
                    continue

                if rep and rep[0] == b"OK":
                    payload = rep[-1] if len(rep) >= 2 else b""
                    if len(payload) >= 12:
                        rx, ry, rz = struct.unpack("<fff", payload[:12])
                        self.get_logger().info(f"[ZMQ:ANGLE] OK rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
                        if self._seq_active:
                            call_angles(self, self.cli_angle, rx, ry, rz, timeout=3.0)
                    else:
                        self.get_logger().warning(f"[ZMQ:ANGLE] payload too small: {len(payload)}")

                time.sleep(max(self.POLL_ANGLE_MS, 10) / 1000.0)
        finally:
            self._close_req_socket_with_monitor(s, mon, "ANGLE")

    def parse_pickpoint_payload(self, payload: bytes):
        if payload is None:
            return None
        if len(payload) >= 96:
            vals = struct.unpack("<12d", payload[:96])
        elif len(payload) >= 48:
            vals = struct.unpack("<12f", payload[:48])
        else:
            return None
        right6 = tuple(float(x) for x in vals[0:6])
        left6  = tuple(float(x) for x in vals[6:12])
        return right6, left6

    def get_pickpoint_loop(self):
        self.get_logger().info("[ZMQ] get_pickpoint_loop started")
        s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK")
        try:
            while self._node_running:
                rep, st = self._req_roundtrip(s, mon, "PICK", [b"PICK_POINT_GET?"], poll_ms=2000)

                if st == "timeout":
                    self.get_logger().warning("[ZMQ:PICK] timeout -> recreate socket")
                    self._close_req_socket_with_monitor(s, mon, "PICK")
                    s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK")
                    time.sleep(0.1)
                    continue

                if (not rep) or rep[0] != b"OK":
                    time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)
                    continue

                payload = rep[-1] if len(rep) >= 2 else b""
                parsed = self.parse_pickpoint_payload(payload)

                if parsed is None:
                    self.get_logger().warning(f"[ZMQ:PICK] payload invalid size={len(payload)}")
                    time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)
                    continue

                right6, left6 = parsed
                self.get_logger().info(f"[ZMQ:PICK] OK R={right6} L={left6}")

                ok = call_objpickpoint(self, self.cli_objpickpoint, right6, left6, timeout=5.0)
                if ok:
                    self._sent_pickpoint = True
                    self.get_logger().info("[FLOW] objpickpoint success (locked)")
                    if not self._seq_active:
                        self._seq_active = True
                        self.get_logger().info("[FLOW] seq_active := True (started by objpickpoint)")

                time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)

        finally:
            self._close_req_socket_with_monitor(s, mon, "PICK")

    # ---------- shutdown ----------
    def destroy_node(self):
        self.get_logger().info("Shutting down SequenceNode...")
        self._node_running = False
        try:
            self._cmd_q.put_nowait(b"\x00")
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/python3

# import threading
# import time
# import struct
# import zmq
# import queue

# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.task import Future

# from std_srvs.srv import Empty
# from dual_arm_msg.srv import Rotblock
# from dual_arm_msg.srv import Objpickpoint


# # ===== 공통 유틸: Future 동기 대기 =====
# def wait_future_blocking(node: Node, fut: Future, timeout: float) -> bool:
#     end_t = time.time() + timeout
#     while rclpy.ok() and (time.time() < end_t):
#         if fut.done():
#             return True
#         time.sleep(0.01)  # 10ms 폴링
#     return fut.done()


# # ===== 서비스 호출 유틸 =====
# def call_empty(node: Node, client, timeout=3.0):
#     if not client.wait_for_service(timeout_sec=timeout):
#         node.get_logger().error(f"[SRV] {client.srv_name} not available")
#         return False

#     fut: Future = client.call_async(Empty.Request())
#     ok = wait_future_blocking(node, fut, timeout)
#     if not ok or fut.result() is None:
#         node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
#         return False

#     node.get_logger().info(f"[SRV] {client.srv_name} → OK")
#     return True


# def call_angles(node: Node, client, rx: float, ry: float, rz: float, timeout=3.0):
#     def normalize_deg(a: float) -> float:
#         a = float(a)
#         if a > 180.0:
#             a -= 360.0
#         return a

#     if not client.wait_for_service(timeout_sec=timeout):
#         node.get_logger().error(f"[SRV] {client.srv_name} not available")
#         return False

#     req = Rotblock.Request()
#     req.r_rx_deg = normalize_deg(rx)
#     req.r_ry_deg = normalize_deg(ry)
#     req.r_rz_deg = normalize_deg(rz)

#     fut: Future = client.call_async(req)
#     ok = wait_future_blocking(node, fut, timeout)
#     if not ok or fut.result() is None:
#         node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
#         return False

#     resp = fut.result()
#     node.get_logger().info(
#         f"[SRV] {client.srv_name} → success={resp.success}, msg='{resp.message}'"
#     )
#     return bool(resp.success)


# def call_objpickpoint(node: Node, client, right: tuple, left: tuple, timeout: float = 3.0):
#     if not client.wait_for_service(timeout_sec=timeout):
#         node.get_logger().error(f"[SRV] {client.srv_name} not available")
#         return False

#     req = Objpickpoint.Request()
#     req.right_x, req.right_y, req.right_z, req.right_rx, req.right_ry, req.right_rz = map(float, right)
#     req.left_x,  req.left_y,  req.left_z,  req.left_rx,  req.left_ry,  req.left_rz  = map(float, left)

#     fut: Future = client.call_async(req)
#     ok = wait_future_blocking(node, fut, timeout)
#     if not ok or fut.result() is None:
#         node.get_logger().error(f"[SRV] {client.srv_name} call failed/timeout")
#         return False

#     resp = fut.result()
#     node.get_logger().info(
#         f"[SRV] {client.srv_name} → success={resp.success}, msg='{resp.message}'"
#     )
#     return bool(resp.success)


# class SequenceNode(Node):
#     # ===== 하드코딩 설정 =====
#     SRV_START_MAINSEQ = "start_mainseq"
#     SRV_STEP4_OK      = "/cam_check_prev"
#     SRV_STEP5_OK      = "cam_check"
#     SRV_FINISH_MAINSEQ= "finish_mainseq"
#     SRV_ANGLE_CMD     = "angle_cmd"
#     SRV_BLOCK_CHECK   = "block_check"
#     SRV_OBJPICKPOINT  = "objpickpoint"

#     ADDR_CALC_CMD_REQ      = "tcp://127.0.0.1:7008"
#     ADDR_ANGLE_GET_REQ     = "tcp://127.0.0.1:7009"
#     ADDR_BLOCK_OUT_REQ     = "tcp://127.0.0.1:7010"
#     ADDR_PICKPOINT_GET_REQ = "tcp://127.0.0.1:7011"

#     POLL_ANGLE_MS     = 100
#     POLL_BLOCK_MS     = 300
#     POLL_PICKPOINT_MS = 100

#     def __init__(self):
#         super().__init__("sequence_node")

#         # ---- ROS Clients/Servers ----
#         self.cli_step_start   = self.create_client(Empty,    self.SRV_START_MAINSEQ)
#         self.cli_angle        = self.create_client(Rotblock, self.SRV_ANGLE_CMD)
#         self.cli_block_out    = self.create_client(Empty,    self.SRV_BLOCK_CHECK)
#         self.cli_objpickpoint = self.create_client(Objpickpoint, self.SRV_OBJPICKPOINT)

#         self.create_service(Empty, self.SRV_STEP4_OK,       self._cb_step4_ok)
#         self.create_service(Empty, self.SRV_STEP5_OK,       self._cb_step5_ok)
#         self.create_service(Empty, self.SRV_FINISH_MAINSEQ, self._cb_step_done)

#         # ---- ZMQ Context ----
#         self._ctx = zmq.Context.instance()

#         # ---- 상태 ----
#         self._seq_active = False
#         self._node_running = True
#         self._seq_completed = False
#         self._start_after_pickpoint = False
#         self._sent_pickpoint = False

#         # cam_check 콜백 -> 큐로 넘김 (콜백에서 소켓 사용 금지)
#         self._cmd_q: "queue.Queue[bytes]" = queue.Queue()

#         # 시작 시 stale flush (임시 소켓)
#         self._flush_stale_angles(max_flush=5)
#         self._flush_stale_pickpoints(max_flush=5)

#         # 스레드 시작
#         threading.Thread(target=self._key_loop, daemon=True).start()
#         threading.Thread(target=self._calc_cmd_worker, daemon=True).start()
#         threading.Thread(target=self.get_angle_loop, daemon=True).start()
#         # threading.Thread(target=self.get_block_out_loop, daemon=True).start()
#         threading.Thread(target=self.get_pickpoint_loop, daemon=True).start()

#         self.get_logger().info("SequenceNode ready. [Enter]=시작/재시작, [ESC]=종료")

#     # ---------- ZMQ helpers ----------
#     def _make_req_socket_with_monitor(self, addr: str, name: str):
#         s = self._ctx.socket(zmq.REQ)
#         s.setsockopt(zmq.LINGER, 0)

#         # REQ 타임아웃/재시도 시 상태 꼬임 완화
#         s.setsockopt(zmq.REQ_RELAXED, 1)
#         s.setsockopt(zmq.REQ_CORRELATE, 1)

#         s.setsockopt(zmq.SNDTIMEO, 2000)
#         s.setsockopt(zmq.RCVTIMEO, 2000)

#         s.connect(addr)

#         mon_addr = f"inproc://mon-{name}-{id(s)}"
#         s.monitor(mon_addr, zmq.EVENT_CONNECTED | zmq.EVENT_CONNECT_DELAYED | zmq.EVENT_DISCONNECTED)
#         mon = self._ctx.socket(zmq.PAIR)
#         mon.connect(mon_addr)

#         self.get_logger().info(f"[ZMQ:{name}] connect -> {addr}")
#         return s, mon

#     def _close_req_socket_with_monitor(self, s, mon, name: str):
#         try:
#             mon.close(0)
#         except Exception:
#             pass
#         try:
#             s.disable_monitor()
#         except Exception:
#             pass
#         try:
#             s.close(0)
#         except Exception:
#             pass
#         self.get_logger().info(f"[ZMQ:{name}] socket closed")

#     def _drain_monitor(self, mon, name: str):
#         # 이벤트가 있을 때만 읽음 (스팸 방지)
#         try:
#             while mon.poll(0, zmq.POLLIN):
#                 evt = mon.recv_multipart()
#                 self.get_logger().info(f"[ZMQ:{name}] MON evt frames={len(evt)}")
#         except Exception as e:
#             self.get_logger().warning(f"[ZMQ:{name}] MON read error: {e}")

#     def _req_roundtrip(self, s, mon, name: str, frames: list[bytes], poll_ms: int = 2000):
#         self._drain_monitor(mon, name)

#         try:
#             self.get_logger().debug(f"[ZMQ:{name}] SEND {frames}")
#             s.send_multipart(frames)
#         except Exception as e:
#             self.get_logger().error(f"[ZMQ:{name}] SEND failed: {e}")
#             return None, "send_fail"

#         try:
#             if s.poll(poll_ms, zmq.POLLIN) == 0:
#                 self.get_logger().warning(f"[ZMQ:{name}] RECV timeout (no reply in {poll_ms}ms)")
#                 return None, "timeout"
#             rep = s.recv_multipart()
#             self.get_logger().debug(f"[ZMQ:{name}] RECV {rep}")
#             return rep, "ok"
#         except Exception as e:
#             self.get_logger().error(f"[ZMQ:{name}] RECV failed: {e}")
#             return None, "recv_fail"

#     # ---------- Flush stale (임시 소켓) ----------
#     def _flush_stale_angles(self, max_flush: int = 5):
#         self.get_logger().info(f"[ZMQ] Flushing stale ANGLE_GET values (max={max_flush})...")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE_FLUSH")
#         try:
#             for i in range(max_flush):
#                 rep, st = self._req_roundtrip(s, mon, "ANGLE_FLUSH", [b"ANGLE_GET?"], poll_ms=800)
#                 if st != "ok" or (not rep) or rep[0] != b"OK":
#                     self.get_logger().info(f"[ZMQ] angle flush done at iter={i}, st={st}, rep={rep}")
#                     break
#                 self.get_logger().warning(f"[ZMQ] flushed stale angle #{i}: frames={len(rep)}")
#         finally:
#             self._close_req_socket_with_monitor(s, mon, "ANGLE_FLUSH")
#         self.get_logger().info("[ZMQ] ANGLE flush finished.")

#     def _flush_stale_pickpoints(self, max_flush: int = 5):
#         self.get_logger().info(f"[ZMQ] Flushing stale PICK_POINT_GET values (max={max_flush})...")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK_FLUSH")
#         try:
#             for i in range(max_flush):
#                 rep, st = self._req_roundtrip(s, mon, "PICK_FLUSH", [b"PICK_POINT_GET?"], poll_ms=800)
#                 if st != "ok" or (not rep) or rep[0] != b"OK":
#                     self.get_logger().info(f"[ZMQ] pick flush done at iter={i}, st={st}, rep={rep}")
#                     break
#                 self.get_logger().warning(f"[ZMQ] flushed stale pickpoint #{i}: frames={len(rep)}")
#         finally:
#             self._close_req_socket_with_monitor(s, mon, "PICK_FLUSH")
#         self.get_logger().info("[ZMQ] PICKPOINT flush finished.")

#     # ---------- Keyboard ----------
#     def _key_loop(self):
#         import sys, tty, termios, select
#         fd = sys.stdin.fileno()
#         old = termios.tcgetattr(fd)
#         try:
#             tty.setcbreak(fd)
#             while self._node_running:
#                 r, _, _ = select.select([sys.stdin], [], [], 0.2)
#                 if not r:
#                     continue
#                 ch = sys.stdin.read(1)
#                 if not ch:
#                     continue
#                 if ord(ch) == 27:  # ESC
#                     self._node_running = False
#                     rclpy.shutdown()
#                     break
#                 if ch in ('\n', '\r'):
#                     if not self._seq_active:
#                         self._seq_active = True
#                         self._sent_pickpoint = False
#                         self._start_after_pickpoint = True
#                         self.get_logger().info("[START] start_mainseq 호출")
#                         # call_empty(self, self.cli_step_start)
#         except Exception as e:
#             self.get_logger().error(f"[KEY] loop error: {e}")
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old)

#     # ---------- ROS callbacks ----------
#     def _cb_step4_ok(self, req, resp):
#         if self._seq_active:
#             self.get_logger().info("[RECV] cam_check_prev → angle_cmd(0,0,0)")
#             ok = call_angles(self, self.cli_angle, 0.0, 0.0, 0.0)
#             if not ok:
#                 self.get_logger().error("[SRV] angle_cmd(0,0,0) failed")
#         return resp

#     def _cb_step5_ok(self, req, resp):
#         if self._seq_active:
#             self.get_logger().info("[RECV] cam_check → enqueue CALC_ANGLE_CMD(True)")
#             try:
#                 self._cmd_q.put_nowait(b"\x01")
#             except Exception as e:
#                 self.get_logger().error(f"[ZMQ] enqueue failed: {e}")
#         return resp

#     def _cb_step_done(self, req, resp):
#         if self._seq_active:
#             self.get_logger().info("[RECV] finish_mainseq → 재시작")
#             self._seq_active = False
#             self._sent_pickpoint = False

#             if self._seq_completed is False:
#                 self._seq_active = True
#                 call_empty(self, self.cli_step_start)
#                 self._seq_completed = True
#         return resp

#     # ---------- ZMQ workers ----------
#     def _calc_cmd_worker(self):
#         self.get_logger().info("[ZMQ] calc_cmd_worker started")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_CALC_CMD_REQ, "CMD")
#         try:
#             while self._node_running:
#                 try:
#                     flag = self._cmd_q.get(timeout=0.2)  # b"\x01" / b"\x00"
#                 except queue.Empty:
#                     continue

#                 rep, st = self._req_roundtrip(s, mon, "CMD", [b"CALC_ANGLE_CMD", flag[:1]], poll_ms=2000)
#                 if st == "timeout":
#                     # timeout이면 소켓 꼬임 방지: 재생성
#                     self.get_logger().warning("[ZMQ:CMD] timeout -> recreate socket")
#                     self._close_req_socket_with_monitor(s, mon, "CMD")
#                     s, mon = self._make_req_socket_with_monitor(self.ADDR_CALC_CMD_REQ, "CMD")
#         finally:
#             self._close_req_socket_with_monitor(s, mon, "CMD")
#             self.get_logger().info("[ZMQ] calc_cmd_worker exited")

#     def get_angle_loop(self):
#         self.get_logger().info("[ZMQ] get_angle_loop started")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE")
#         try:
#             while self._node_running:
#                 rep, st = self._req_roundtrip(s, mon, "ANGLE", [b"ANGLE_GET?"], poll_ms=2000)

#                 if st == "timeout":
#                     self.get_logger().warning("[ZMQ:ANGLE] timeout -> recreate socket")
#                     self._close_req_socket_with_monitor(s, mon, "ANGLE")
#                     s, mon = self._make_req_socket_with_monitor(self.ADDR_ANGLE_GET_REQ, "ANGLE")
#                     time.sleep(0.1)
#                     continue

#                 if rep and rep[0] == b"OK":
#                     payload = rep[-1] if len(rep) >= 2 else b""
#                     if len(payload) >= 12:
#                         rx, ry, rz = struct.unpack("<fff", payload[:12])
#                         self.get_logger().info(f"[ZMQ:ANGLE] OK rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
#                         if self._seq_active:
#                             call_angles(self, self.cli_angle, rx, ry, rz, timeout=3.0)
#                     else:
#                         self.get_logger().warning(f"[ZMQ:ANGLE] payload too small: {len(payload)}")

#                 time.sleep(max(self.POLL_ANGLE_MS, 10) / 1000.0)
#         finally:
#             self._close_req_socket_with_monitor(s, mon, "ANGLE")

#     def get_block_out_loop(self):
#         self.get_logger().info("[ZMQ] get_block_out_loop started")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_BLOCK_OUT_REQ, "BLOCK")
#         try:
#             while self._node_running:
#                 rep, st = self._req_roundtrip(s, mon, "BLOCK", [b"BLOCK_OUT?"], poll_ms=2000)

#                 if st == "timeout":
#                     self.get_logger().warning("[ZMQ:BLOCK] timeout -> recreate socket")
#                     self._close_req_socket_with_monitor(s, mon, "BLOCK")
#                     s, mon = self._make_req_socket_with_monitor(self.ADDR_BLOCK_OUT_REQ, "BLOCK")
#                     time.sleep(0.1)
#                     continue

#                 if rep and rep[0] == b"OK":
#                     payload = rep[-1] if len(rep) >= 2 else b""
#                     val = (len(payload) >= 1 and payload[0] != 0)
#                     if val:
#                         self.get_logger().info("[ZMQ:BLOCK] OK (edge)")
#                         if self._seq_active:
#                             call_empty(self, self.cli_block_out, timeout=3.0)

#                 time.sleep(max(self.POLL_BLOCK_MS, 10) / 1000.0)
#         finally:
#             self._close_req_socket_with_monitor(s, mon, "BLOCK")

#     def parse_pickpoint_payload(self, payload: bytes):
#         if payload is None:
#             return None
#         if len(payload) >= 96:
#             vals = struct.unpack("<12d", payload[:96])
#         elif len(payload) >= 48:
#             vals = struct.unpack("<12f", payload[:48])
#         else:
#             return None
#         right6 = tuple(float(x) for x in vals[0:6])
#         left6  = tuple(float(x) for x in vals[6:12])
#         return right6, left6

#     def get_pickpoint_loop(self):
#         self.get_logger().info("[ZMQ] get_pickpoint_loop started")
#         s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK")
#         try:
#             while self._node_running:
#                 rep, st = self._req_roundtrip(s, mon, "PICK", [b"PICK_POINT_GET?"], poll_ms=2000)

#                 if st == "timeout":
#                     self.get_logger().warning("[ZMQ:PICK] timeout -> recreate socket")
#                     self._close_req_socket_with_monitor(s, mon, "PICK")
#                     s, mon = self._make_req_socket_with_monitor(self.ADDR_PICKPOINT_GET_REQ, "PICK")
#                     time.sleep(0.1)
#                     continue

#                 # OK 아닌 경우는 절대 아래로 못 내려가게
#                 if (not rep) or rep[0] != b"OK":
#                     time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)
#                     continue

#                 payload = rep[-1] if len(rep) >= 2 else b""
#                 parsed = self.parse_pickpoint_payload(payload)

#                 # 파싱 실패면 절대 아래로 못 내려가게
#                 if parsed is None:
#                     self.get_logger().warning(f"[ZMQ:PICK] payload invalid size={len(payload)}")
#                     time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)
#                     continue

#                 # ★ right6/left6는 여기서만 정의됨
#                 right6, left6 = parsed
#                 self.get_logger().info(f"[ZMQ:PICK] OK R={right6} L={left6}")

#                 ok = call_objpickpoint(self, self.cli_objpickpoint, right6, left6, timeout=5.0)
#                 if ok:
#                     self._sent_pickpoint = True
#                     self.get_logger().info("[FLOW] objpickpoint success (locked)")
#                     if not self._seq_active:
#                         self._seq_active = True
#                         self.get_logger().info("[FLOW] seq_active := True (started by objpickpoint)")

#                 time.sleep(max(self.POLL_PICKPOINT_MS, 10) / 1000.0)

#         finally:
#             self._close_req_socket_with_monitor(s, mon, "PICK")


#     # ---------- shutdown ----------
#     def destroy_node(self):
#         self.get_logger().info("Shutting down SequenceNode...")
#         self._node_running = False
#         try:
#             self._cmd_q.put_nowait(b"\x00")
#         except Exception:
#             pass
#         return super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SequenceNode()
#     try:
#         executor = MultiThreadedExecutor(num_threads=3)
#         executor.add_node(node)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
