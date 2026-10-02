#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Trigger

class URStartupNode(Node):
    def __init__(self):
        super().__init__('ur_startup_node')

        self.robot_ip = "192.168.0.5"
        self.ur_type = "ur3e"
        self.program_name = "moveit.urp"

        # 서비스 클라이언트 준비
        self.load_client = self.create_client(Load, '/dashboard_client/load_program')
        self.play_client = self.create_client(Trigger, '/dashboard_client/play')

        self.get_logger().info("✅ UR startup sequence initiated...")

        # 1) ur_dashboard_client.launch 실행
        self.run_launch(
            "ur_robot_driver",
            "ur_dashboard_client.launch.py",
            [f"robot_ip:={self.robot_ip}"]
        )
        time.sleep(3)  # 노드 올라올 시간 확보

        # 2) ur_control.launch 실행
        self.run_launch(
            "ur_robot_driver",
            "ur_control.launch.py",
            [
                f"ur_type:={self.ur_type}",
                f"robot_ip:={self.robot_ip}",
                "launch_rviz:=false"
            ]
        )
        time.sleep(5)  # 컨트롤러 초기화 대기

        # 3) Load Program → 4) Play Program 순차 실행
        self.call_load_program()

    def run_launch(self, package, launch_file, args=None):
        """ros2 launch 실행"""
        cmd = ["ros2", "launch", package, launch_file]
        if args:
            cmd += args
        self.get_logger().info(f"🚀 Launching: {' '.join(cmd)}")
        subprocess.Popen(cmd)  # 백그라운드 실행

    def call_load_program(self):
        """moveit.urp 프로그램 로드"""
        self.get_logger().info("⏳ Waiting for /dashboard_client/load_program service...")
        self.load_client.wait_for_service()

        req = Load.Request()
        req.filename = self.program_name

        future = self.load_client.call_async(req)
        future.add_done_callback(self.after_load_program)

    def after_load_program(self, future):
        """프로그램 로드 후 play 호출"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("✅ Program loaded successfully!")
                self.call_play()
            else:
                self.get_logger().error(f"❌ Failed to load program: {response.answer}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def call_play(self):
        """로드된 프로그램 실행"""
        self.get_logger().info("⏳ Waiting for /dashboard_client/play service...")
        self.play_client.wait_for_service()

        req = Trigger.Request()
        future = self.play_client.call_async(req)
        future.add_done_callback(self.after_play)

    def after_play(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("✅ Program started successfully!")
            else:
                self.get_logger().error(f"❌ Failed to start program: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Play service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = URStartupNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
