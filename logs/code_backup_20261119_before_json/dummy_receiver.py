#!/usr/bin/env python3
"""더미 수신기 - a_mission / a_remission 이 보낸 사진(detection_image)과 데이터(capture_info)를 받아 확인한다.

실제 서버가 생기기 전 테스트용 (v2 이후 버전과 a_remission.py 의 송신 확인).
참고: day2/2_1_e_capture_comp_image.py (CompressedImage 수신 · 변환), 2_0_d (String 수신), 2_4_e (CSV 저장)

실행: ros2 run robot_a_navigation dummy_receiver --ros-args -r __ns:=/robot5      (끝낼 때 Ctrl+C)
결과: ~/farmmate_ws/logs/a_mission/received/<번호>_<섹터>.jpg     받은 사진
      ~/farmmate_ws/logs/a_mission/<시각>_received.csv           번호·섹터·모드·사진 크기·용량·짝 맞음 여부
"""

import csv
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/a_mission')
IMAGE_TOPIC = 'detection_image'
DATA_TOPIC = 'capture_info'
FIELDS = ['id', 'time', 'sector', 'x', 'y', 'yaw', 'mode', 'width', 'height', 'jpeg_bytes', 'paired', 'file']


class DummyReceiver(Node):

    def __init__(self):
        super().__init__('dummy_receiver')
        self.bridge = CvBridge()
        self.images = {}   # 번호 → (사진, 용량)  : 사진이 먼저 오면 여기 보관
        self.count = 0
        self.save_dir = os.path.join(LOG_DIR, 'received')
        os.makedirs(self.save_dir, exist_ok=True)
        self.csv_path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_received.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()
        self.create_subscription(CompressedImage, IMAGE_TOPIC, self.image_callback, 10)
        self.create_subscription(String, DATA_TOPIC, self.data_callback, 10)
        self.get_logger().info(f'수신 대기 중... ({IMAGE_TOPIC} · {DATA_TOPIC}) 기록: {self.csv_path}')

    def image_callback(self, msg):
        """사진 수신 (2_1_e): 번호는 header.frame_id 에 들어 있음."""
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.images[msg.header.frame_id] = (frame, len(msg.data))

    def data_callback(self, msg):
        """데이터 수신 (2_0_d): '번호,시각,섹터,x,y,yaw,모드' → 같은 번호의 사진과 짝을 맞춰 저장."""
        parts = msg.data.split(',')
        row = dict(zip(FIELDS[:7], parts))
        frame, size = self.images.pop(row['id'], (None, 0))
        row.update(paired=frame is not None, jpeg_bytes=size, width=None, height=None, file=None)
        if frame is not None:
            row.update(height=frame.shape[0], width=frame.shape[1],
                       file=os.path.join(self.save_dir, f"{row['id']}_{row['sector']}.jpg"))
            cv2.imwrite(row['file'], frame)
        self.writer.writerow(row)
        self.csv_file.flush()
        self.count += 1
        self.get_logger().info(f"받음 #{row['id']} {row['sector']} ({row['mode']}) 위치=({row['x']}, {row['y']}) · "
                               f"사진 {row['width']}x{row['height']} {size}바이트 · 짝 맞음={row['paired']}")


def main():
    rclpy.init()
    node = DummyReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f'받은 개수: {node.count} · 저장: {node.csv_path}')
    node.csv_file.close()


if __name__ == '__main__':
    main()
