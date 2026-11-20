#!/usr/bin/env python3
"""더미 재순찰 서버 - a_remission 이 요청하면 재순찰 목록(서비스 remission_list)을 돌려준다.

실제 서버가 생기기 전 테스트용. a_remission 이 보낸 capture_info(mode=recheck)를 받으면 그 구역을 목록에서 뺀다.
첫 목록이 다 끝나면 SECOND_AFTER 초 뒤에 두 번째 목록을 내놓아서 "30초 대기 중 새 목록" 흐름도 확인할 수 있다.
참고: ROS 2 튜토리얼 "Writing a simple service and client (Python)" (서비스 서버), day2/2_0_d (String 수신),
      day2/2_4_e (CSV 저장)

실행: ros2 run robot_a_navigation dummy_remission_server --ros-args -r __ns:=/robot5      (끝낼 때 Ctrl+C)
결과: ~/farmmate_ws/logs/a_remission/<시각>_dummy_remission_server.csv   요청 시각·돌려준 구역·재확인 받은 구역
"""

import csv
import math
import os
import time

import rclpy
from farm_interfaces.msg import ZonePose
from farm_interfaces.srv import RemissionList
from rclpy.node import Node
from std_msgs.msg import String

# ===== 돌려줄 목록 (여기만 바꾸면 됨): (구역 이름, x, y, yaw 라디안) =====
LISTS = [
    ('R_TEST_001', [('A_1', -0.57, -0.15, math.pi), ('A_2', -1.89, 0.037, 0.0)]),
    ('R_TEST_002', [('A_1', -0.57, -0.15, math.pi)]),
]
SECOND_AFTER = 10.0      # 앞 목록이 끝나고 다음 목록을 내놓을 때까지 걸리는 시간(초)
DATA_TOPIC = 'capture_info'
LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/a_remission')


class DummyRemissionServer(Node):

    def __init__(self):
        super().__init__('dummy_remission_server')
        self.index = 0             # 지금 내놓는 목록 번호
        self.remaining = [z[0] for z in LISTS[0][1]]
        self.emptied_at = None     # 지금 목록이 다 끝난 시각
        os.makedirs(LOG_DIR, exist_ok=True)
        self.csv_path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_dummy_remission_server.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'event', 'list_id', 'zones'])
        self.csv_file.flush()
        self.create_service(RemissionList, 'remission_list', self.list_callback)
        self.create_subscription(String, DATA_TOPIC, self.data_callback, 10)
        self.get_logger().info(f'요청 대기 중... (서비스 remission_list) 첫 목록: {self.remaining}')

    def log(self, event, zones):
        self.writer.writerow([time.strftime('%H:%M:%S'), event, LISTS[min(self.index, len(LISTS) - 1)][0],
                              ' '.join(zones)])
        self.csv_file.flush()

    def list_callback(self, request, response):
        """목록 요청 → 아직 재확인하지 않은 구역을 돌려줌."""
        # 지금 목록이 끝나고 SECOND_AFTER 초가 지났으면 다음 목록으로 넘어감
        if (self.emptied_at is not None and self.index + 1 < len(LISTS)
                and time.time() - self.emptied_at > SECOND_AFTER):
            self.index += 1
            self.remaining = [z[0] for z in LISTS[self.index][1]]
            self.emptied_at = None
            self.get_logger().info(f'다음 목록 준비: {LISTS[self.index][0]} {self.remaining}')
        list_id, zones = LISTS[self.index]
        response.list_id = list_id
        response.zones = [ZonePose(zone=name, x=x, y=y, yaw=yaw) for name, x, y, yaw in zones
                          if name in self.remaining]
        self.get_logger().info(f'요청 받음({request.robot_id}) → {list_id} {[z.zone for z in response.zones]}')
        self.log('request', [z.zone for z in response.zones])
        return response

    def data_callback(self, msg):
        """재확인 결과 수신 ('번호,시각,섹터,x,y,yaw,모드') → mode=recheck 면 목록에서 뺌."""
        parts = msg.data.split(',')
        sector, mode = parts[2], parts[-1]
        if mode == 'recheck' and sector in self.remaining:
            self.remaining.remove(sector)
            self.get_logger().info(f'재확인 받음: {sector} → 남은 구역 {self.remaining}')
            self.log('recheck', [sector])
            if not self.remaining:
                self.emptied_at = time.time()


def main():
    rclpy.init()
    node = DummyRemissionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f'저장: {node.csv_path}')
    node.csv_file.close()


if __name__ == '__main__':
    main()
