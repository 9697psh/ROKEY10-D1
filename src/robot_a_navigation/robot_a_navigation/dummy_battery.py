#!/usr/bin/env python3
"""더미 배터리 - 키보드로 넣은 잔량(%)을 battery_state_dummy 로 계속 발행한다.

배터리 복귀 · 재개 테스트용 (v4_battery.py · a_mission.py). 실제 로봇의 battery_state 와 섞이지 않게
다른 토픽 이름을 쓰므로, 테스트할 때는 v4_battery.py 의 BATTERY_TOPIC 을 'battery_state_dummy' 로 바꾼다.
참고: day2/2_0_c (타이머로 발행), day3/3_1_e (input 으로 값 받기), day3/3_1_f (BatteryState · qos_profile_sensor_data)

실행: ros2 run robot_a_navigation dummy_battery --ros-args -r __ns:=/robot5
사용: 숫자를 입력하고 Enter (예: 15 → 15%, 96 → 96%). 처음 값은 START_PERCENT. 끝낼 때 Ctrl+C
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

BATTERY_TOPIC = 'battery_state_dummy'
START_PERCENT = 50.0     # 처음 잔량(%)


class DummyBattery(Node):

    def __init__(self):
        super().__init__('dummy_battery')
        self.percent = START_PERCENT
        self.publisher = self.create_publisher(BatteryState, BATTERY_TOPIC, qos_profile_sensor_data)
        self.create_timer(1.0, self.timer_callback)   # 1초마다 발행 (2_0_c)

    def timer_callback(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self.percent / 100.0          # BatteryState 는 0.0 ~ 1.0
        self.publisher.publish(msg)


def spin(node):
    try:
        rclpy.spin(node)
    except Exception:   # 종료할 때 나는 예외는 무시
        pass


def main():
    rclpy.init()
    node = DummyBattery()
    threading.Thread(target=spin, args=(node,), daemon=True).start()
    print(f'발행 중: {BATTERY_TOPIC} = {node.percent}%  (숫자 입력 후 Enter 로 변경, Ctrl+C 로 종료)')
    try:
        while True:
            text = input('배터리 % > ').strip()
            try:
                node.percent = min(100.0, max(0.0, float(text)))
                print(f'→ {node.percent}% 로 발행')
            except ValueError:
                print('숫자만 입력 (예: 15)')
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == '__main__':
    main()
