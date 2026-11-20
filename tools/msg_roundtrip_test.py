#!/usr/bin/env python3
"""farm_interfaces 메시지 왕복 시험 (발행 → 구독) · 검증 수치 저장.

참고 코드 (github.com/kimandreas/to_students):
  - day2/2_0_c_data_publisher.py  : 타이머로 메시지 발행하는 구조
  - day2/2_0_d_data_subscriber.py : 구독 콜백에서 받은 값 확인하는 구조
새로 작성: farm_interfaces 4개 메시지 채우기, QoS(계획안), 지연(ms)·크기(bytes)·필드 일치 계산, CSV·JSON 저장

실행 (다른 사람 시험과 섞이지 않게 시험용 도메인 + 이 PC 안에서만):
    source /opt/ros/jazzy/setup.bash && source ~/farmmate_ws/install/setup.bash
    ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 ~/farmmate_ws/tools/msg_roundtrip_test.py --count 100 --tasks 1

결과: ~/farmmate_ws/logs/interfaces/<시각>_msg_roundtrip.csv · _summary.json
"""

import argparse
import csv
import json
import os
import statistics
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from farm_interfaces.msg import SurveyComplete, TargetObject, TaskAck, TaskResult, WorkTarget, WorkTargetArray
from sensor_msgs.msg import CompressedImage

LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/interfaces')


def qos(depth, transient):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)


# 토픽 · 타입 · QoS (drawio 02 페이지 계획안)
TOPICS = [
    ('/farm/work_targets', WorkTargetArray, qos(10, False)),
    ('/farm/task_ack', TaskAck, qos(10, False)),
    ('/farm/sector_complete', TaskResult, qos(10, True)),
    ('/farm/survey_complete', SurveyComplete, qos(1, True)),
]


def make_jpeg(path, w, h):
    """시험용 사진: 파일이 있으면 그대로, 없으면 그라데이션+잡음 JPEG 생성."""
    if path:
        with open(path, 'rb') as f:
            return f.read(), f'file:{os.path.basename(path)}'
    import cv2
    x = np.linspace(0, 255, w, dtype=np.float32)
    img = np.stack([np.tile(x, (h, 1)), np.tile(x[::-1], (h, 1)), np.full((h, w), 120, np.float32)], axis=2)
    img += np.random.default_rng(0).normal(0, 12, img.shape)
    ok, buf = cv2.imencode('.jpg', np.clip(img, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    return buf.tobytes(), f'synthetic {w}x{h} q90'


def build(msg_type, seq, now, jpeg, n_tasks):
    if msg_type is WorkTargetArray:
        m = WorkTargetArray()
        m.header.stamp, m.header.frame_id, m.mission_id = now, 'map', 'M_TEST'
        for k in range(n_tasks):
            t = WorkTarget()
            t.task_id, t.revision, t.type, t.sector_id = f'T{seq:04d}_{k}', 1, 'new', f'sector_{k + 1}'
            t.goal.header.frame_id = 'map'
            t.goal.pose.position.x, t.goal.pose.position.y, t.goal.pose.orientation.w = 1.0 + k, -0.5, 1.0
            t.targets = [TargetObject(target_id=f't{i}', cls='ripe', conf=0.9, box=[10, 20, 30, 40]) for i in range(8)]
            t.goal_count = len(t.targets)
            t.ref_photo_id = f'P{seq:04d}_{k}'
            t.ref_photo = CompressedImage(format='jpeg', data=jpeg)
            t.ref_labeled_photo = CompressedImage(format='jpeg', data=jpeg)
            m.tasks.append(t)
        return m
    if msg_type is TaskAck:
        m = TaskAck(mission_id='M_TEST', task_id=f'T{seq:04d}_0', revision=1, accepted=(seq % 2 == 0),
                    reason='' if seq % 2 == 0 else 'PHOTO_MISSING')
    elif msg_type is TaskResult:
        m = TaskResult(event_id=f'E{seq:04d}', mission_id='M_TEST', task_id=f'T{seq:04d}_0', sector_id='sector_1',
                       revision=1, completion_mode=TaskResult.COMPLETION_AUTO, remaining_count=2, last_state='HOLD',
                       ref_photo_id='P0', final_photo_id='P1')
        m.decision_stamp = now
    else:
        m = SurveyComplete(mission_id='M_TEST', surveyed_sectors=4, tasks_issued=seq, no_target_sectors=1)
    m.header.stamp, m.header.frame_id = now, 'map'
    return m


def fields_ok(sent, got):
    """보낸 값과 받은 값이 같은지 (사진 바이트 길이 포함)."""
    if isinstance(sent, WorkTargetArray):
        return (len(sent.tasks) == len(got.tasks) and all(
            a.task_id == b.task_id and a.revision == b.revision and len(a.targets) == len(b.targets)
            and list(a.targets[0].box) == list(b.targets[0].box) and len(a.ref_photo.data) == len(b.ref_photo.data)
            for a, b in zip(sent.tasks, got.tasks)))
    return serialize_message(sent) == serialize_message(got)


class Tester(Node):
    def __init__(self, msg_type, topic, q):
        super().__init__('farm_msg_roundtrip_' + msg_type.__name__.lower())
        self.pub = self.create_publisher(msg_type, topic, q)
        self.sub = self.create_subscription(msg_type, topic, self.cb, q)
        self.recv = {}  # seq -> (recv_time_ns, msg)

    def cb(self, msg):  # 2_0_d 구독 콜백 구조
        self.recv[self.key(msg)] = (time.time_ns(), msg)

    @staticmethod
    def key(msg):
        if isinstance(msg, WorkTargetArray):
            return msg.tasks[0].task_id if msg.tasks else ''
        if isinstance(msg, SurveyComplete):
            return str(msg.tasks_issued)
        return msg.task_id


def run_one(topic, msg_type, q, args, jpeg):
    node = Tester(msg_type, topic, q)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t0 = time.time()
    while node.pub.get_subscription_count() < 1 and time.time() - t0 < 5.0:  # 구독 연결 대기
        ex.spin_once(timeout_sec=0.05)
    sent, rows = {}, []
    period = 1.0 / args.rate
    for seq in range(args.count):
        now = node.get_clock().now().to_msg()
        m = build(msg_type, seq, now, jpeg, args.tasks)
        k = Tester.key(m)
        sent[k] = (time.time_ns(), m, len(serialize_message(m)))
        node.pub.publish(m)
        end = time.time() + period
        while time.time() < end:
            ex.spin_once(timeout_sec=max(0.0, end - time.time()))
    end = time.time() + 2.0  # 늦게 오는 메시지 대기
    while time.time() < end and len(node.recv) < len(sent):
        ex.spin_once(timeout_sec=0.05)
    for k, (ts, m, size) in sent.items():
        r = node.recv.get(k)
        rows.append({'msg_type': msg_type.__name__, 'topic': topic, 'key': k, 'size_bytes': size,
                     'received': r is not None,
                     'latency_ms': round((r[0] - ts) / 1e6, 3) if r else None,
                     'fields_ok': fields_ok(m, r[1]) if r else None})
    ex.remove_node(node)
    node.destroy_node()
    return rows


def summarize(rows):
    lat = [r['latency_ms'] for r in rows if r['received']]
    return {'sent': len(rows), 'received': len(lat), 'recv_rate': round(len(lat) / len(rows), 4) if rows else None,
            'latency_ms_mean': round(statistics.mean(lat), 3) if lat else None,
            'latency_ms_p95': round(sorted(lat)[int(0.95 * (len(lat) - 1))], 3) if lat else None,
            'latency_ms_max': round(max(lat), 3) if lat else None,
            'size_bytes': rows[0]['size_bytes'] if rows else None,
            'fields_mismatch': sum(1 for r in rows if r['received'] and not r['fields_ok'])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=100)
    ap.add_argument('--rate', type=float, default=10.0, help='발행 Hz')
    ap.add_argument('--tasks', type=int, default=1, help='WorkTargetArray 안 작업 수')
    ap.add_argument('--photo', default='', help='시험용 JPEG 파일 (없으면 1280x720 합성)')
    ap.add_argument('--build-seconds', type=float, default=None)
    args = ap.parse_args()

    jpeg, photo_desc = make_jpeg(args.photo, 1280, 720)
    rclpy.init()
    rows = []
    for topic, msg_type, q in TOPICS:
        r = run_one(topic, msg_type, q, args, jpeg)
        rows += r
        s = summarize(r)
        print(f"{msg_type.__name__:16s} {topic:24s} 수신 {s['received']}/{s['sent']} · 지연 평균 {s['latency_ms_mean']} ms "
              f"p95 {s['latency_ms_p95']} 최대 {s['latency_ms_max']} · 크기 {s['size_bytes']} B · 필드 불일치 {s['fields_mismatch']}")
    rclpy.shutdown()

    stamp = time.strftime('%Y%m%d_%H%M%S')
    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, f'{stamp}_msg_roundtrip.csv')
    json_path = os.path.join(LOG_DIR, f'{stamp}_msg_roundtrip_summary.json')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    summary = {'run_start': time.strftime('%Y-%m-%d %H:%M:%S'), 'count': args.count, 'rate_hz': args.rate,
               'tasks_per_array': args.tasks, 'photo': photo_desc, 'photo_bytes': len(jpeg),
               'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'), 'rmw': os.environ.get('RMW_IMPLEMENTATION', 'default'),
               'build_seconds': args.build_seconds, 'note': '같은 PC 안 시험 (PC 간 네트워크 지연은 통합 시험에서 별도 측정)',
               'by_type': {}}
    for topic, msg_type, _ in TOPICS:
        summary['by_type'][msg_type.__name__] = {'topic': topic, **summarize([r for r in rows if r['topic'] == topic])}
    with open(json_path, 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print('저장:', csv_path)
    print('저장:', json_path)


if __name__ == '__main__':
    main()
