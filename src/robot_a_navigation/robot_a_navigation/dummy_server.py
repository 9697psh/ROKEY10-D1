#!/usr/bin/env python3
"""더미 서버 - a_mission 에 시작 명령(mission_cmd 액션)을 보내고 진행 상황 · 결과를 출력한다.

실제 서버가 생기기 전 테스트용. a_mission 쪽에는 더미 코드가 없고, 이 파일이 서버 역할을 대신한다.
참고: ROS 2 튜토리얼 "Writing an action server and client (Python)" 의 클라이언트 구조,
      day3/3_2 (액션 feedback · result 콜백), day2/2_4_e (CSV 저장)

실행: ros2 run robot_a_navigation dummy_server --ros-args -r __ns:=/robot5
결과: ~/farmmate_ws/logs/a_mission/<시각>_dummy_server.csv (수락까지 걸린 시간, feedback 수, 전체 시간, 결과)
"""

import csv
import os
import time

import rclpy
from farm_interfaces.action import MissionCmd
from rclpy.action import ActionClient
from rclpy.node import Node

# ===== 보낼 명령 (여기만 바꾸면 됨) =====
MISSION_ID = 'M_TEST_001'
ZONES = ['A01', 'A02', 'A03', 'A04']
LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/a_mission')

STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}  # action_msgs/GoalStatus


class DummyServer(Node):

    def __init__(self):
        super().__init__('dummy_server')
        self.client = ActionClient(self, MissionCmd, 'mission_cmd')
        self.t_send = None
        self.accept_s = None
        self.feedback_count = 0
        self.last_state = None
        self.finished = False

    def send_start(self):
        """1. 시작 명령 보내기."""
        self.get_logger().info('a_mission 액션 서버를 기다리는 중...')
        self.client.wait_for_server()
        goal = MissionCmd.Goal(mission_id=MISSION_ID, zones=ZONES)
        self.t_send = time.time()
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'시작 명령 전송: mission_id={MISSION_ID} · zones={ZONES}')

    def goal_response_callback(self, future):
        """2. a_mission 이 명령을 받았는지 확인."""
        goal_handle = future.result()
        self.accept_s = time.time() - self.t_send
        if not goal_handle.accepted:
            self.get_logger().error('명령이 거절됨 (a_mission 이 이미 임무 중일 수 있음)')
            self.finished = True
            return
        self.get_logger().info(f'명령 수락됨 ({self.accept_s:.2f}초)')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, msg):
        """3. 진행 상황 수신 (1초마다 오지만 바뀔 때만 출력)."""
        fb = msg.feedback
        self.feedback_count += 1
        state = (fb.state, fb.current_zone, fb.visited)
        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(f'진행: {fb.state} {fb.current_zone} (도착 {fb.visited}/{fb.total})')

    def result_callback(self, future):
        """4. 임무 결과 수신 → 출력하고 CSV 로 저장."""
        result = future.result().result
        row = {'mission_id': MISSION_ID, 'status': STATUS_NAMES.get(future.result().status, future.result().status),
               'success': result.success, 'message': result.message, 'visited': result.visited,
               'total': result.total, 'accept_s': round(self.accept_s, 2),
               'feedback_count': self.feedback_count, 'total_s': round(time.time() - self.t_send, 2)}
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_dummy_server.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(f'결과: {row}')
        self.get_logger().info(f'저장: {path}')
        self.finished = True


def main():
    rclpy.init()
    node = DummyServer()
    node.send_start()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
