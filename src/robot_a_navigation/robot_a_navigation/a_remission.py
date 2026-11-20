#!/usr/bin/env python3
"""Re-survey Manager (로봇 A 재순찰, a_remission.py) - 서버가 준 재순찰 목록을 돌며 캡처 · 송신.

설계도 ② 재탐색: 받은 데이터 있음? → 목록 저장 → 순찰 좌표 이동 → 정지 후 사진 캡처 → 서버 전송(mode=recheck)
                → 남은 좌표? → 없으면 30초 자리에서 수신 대기 → 그래도 없으면 docking station 이동 · 대기
목록 받는 방법: 서비스 remission_list (farm_interfaces/srv/RemissionList) 를 로봇이 필요할 때 요청
               (interfaces/ZonePose.msg · RemissionList.srv 를 farm_interfaces 에 추가해 빌드)
참고: day3/3_1_a·3_1_f (도킹 → undock → 초기 위치), 3_1_c (순회·dock), day2/2_1_d·2_0_a·2_0_c (캡처·발행),
      day2/2_4_e (CSV), ROS 2 튜토리얼 "Writing a simple service and client (Python)" (서비스 요청)

실행 순서: localization → nav2 → 이 파일 (RViz 2D Pose Estimate 는 필요 없음)
    ros2 run robot_a_navigation a_remission --ros-args -r __ns:=/robot5
목록 서버(더미): ros2 run robot_a_navigation dummy_remission_server --ros-args -r __ns:=/robot5
결과 (~/farmmate_ws/logs/a_remission/):
  <시각>_remission_metrics.csv  목록 ID · 지점·시도별 시간, 위치/방향 오차, 캡처 시간, 발행 여부
  <시각>_start_metrics.csv      출발 준비 수치 (목록 대기 시간 포함)
  images/                       캡처한 사진
아직 없는 것: 배터리 복귀(a_mission.py 의 기능 4), 3회 실패 시 이전 좌표로 이동 (지금은 건너뜀)
"""

# [설명] 읽는 순서: 설정값 → 위치 측정 함수 → 출발 준비 → 캡처 · 발행 → 목록 요청(request_list) → main()
#        위치 측정 · 출발 준비 · 캡처 · 발행은 a_mission.py 와 같은 코드다. 이 파일에만 있는 것은 request_list() 와 main() 이다.
# [설명] 주석 끝의 '2_1_d', '3_1_f' 같은 표시는 그 줄을 가져온 강의 코드 파일이다 (to_students day2 · day3).
# [주의] 실제 로봇 실행은 11/19 16:08 에 1회뿐이다: 목록 요청 · 수신은 됐지만 이동은 6회 모두 실패했다 (원인 확인 전).

import csv
import json
import math
import os
import sys
import time

import cv2                              # 2_1_d
import rclpy
from cv_bridge import CvBridge          # 2_1_d, 2_0_a
from farm_interfaces.srv import RemissionList
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node             # 2_1_d, 2_0_a, 2_0_c
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image   # 2_1_d (Image), 2_1_e (CompressedImage)
from std_msgs.msg import String         # 2_0_c
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# [설명] 출발 위치를 코드에 고정한다. 항상 도킹 자리에서 출발하므로 RViz 에서 위치를 찍지 않아도 된다.
#        맵을 새로 만들면 원점이 달라질 수 있으니 이 값도 다시 확인한다.
INITIAL_POSITION = [0.0, 0.0]                  # undock 직후 위치 (항상 도킹 자리에서 출발)
INITIAL_DIRECTION = TurtleBot4Directions.SOUTH
# [TODO] 설계도 ②는 3회 실패 시 '전 위치 / docking station 으로 이동' 이지만, 지금은 그 지점을 건너뛴다.
MAX_TRIES = 3            # 지점마다 최대 시도 (3회 실패 시 건너뜀)
# [설명] 11/19 v2 실행에서 실패 직후 바로 재시도해 짧은 통신 지연에 3회를 모두 써 버렸다 → 재시도 전에 쉰다.
RETRY_WAIT = 3.0         # 실패 후 다시 시도하기 전 대기(초) - 통신·위치 지연이 풀릴 시간
LOCALIZE_TIMEOUT = 30.0  # amcl 위치를 기다리는 최대 시간(초)
LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/a_remission')

# ===== 재순찰 목록 설정 =====
ROBOT_ID = 'robot_a'
# [설명] 서버가 보내 주기를 기다리지 않고 로봇이 필요할 때 물어본다 (Service).
#        서버가 늦게 켜져도 목록을 놓치지 않는다.
POLL_EVERY = 5.0         # 목록을 다시 물어보는 간격(초)
WAIT_FOR_LIST = 30.0     # 목록을 다 돌고 자리에서 기다리는 시간(초) (설계도: 30초)
# 서버가 이 시간 안에 답이 없으면 '목록 없음' 과 같게 처리한다 (request_list 참고).
SERVICE_TIMEOUT = 3.0    # 서버 응답을 기다리는 최대 시간(초)
RUN_ONCE = False         # True: 한 번 돌고 도킹하면 종료 (테스트용) · False: 도킹 후 다시 목록 대기

# ===== 캡처 · 송신 설정 =====
IMAGE_DIR = os.path.join(LOG_DIR, 'images')    # 캡처 사진 저장 폴더
# [설명] 토픽 이름 앞에 / 를 붙이지 않는다 → 실행할 때 준 네임스페이스(__ns:=/robot5)가 자동으로 붙는다.
#        로봇 번호가 바뀌어도 코드는 그대로다 (IMAGE_TOPIC · DATA_TOPIC 등 다른 이름도 같은 방식).
CAMERA_TOPIC = 'oakd/rgb/preview/image_raw'    # 2_1_d 토픽 (앞에 / 없음 → /robot5/oakd/... 로 붙음)
# [설명] robot5 의 preview 영상은 이미 352x352 다 (11/19 확인). resize 는 다른 크기가 와도 맞추기 위한 안전장치.
IMAGE_SIZE = 352                               # 설계도: 352x352
IMAGE_TOPIC = 'detection_image'                # 설계도: 서버로 보내는 사진
DATA_TOPIC = 'capture_info'                    # 설계도: 서버로 보내는 번호·시각·섹터·위치
CAPTURE_TIMEOUT = 3.0                          # 사진을 기다리는 최대 시간(초)

# [설명] amcl_callback 이 채우는 값. 초기 위치가 반영됐는지 확인할 때와, TF 를 못 읽을 때의 예비 값으로 쓴다.
latest_pose = {}  # 마지막 amcl_pose: x, y, yaw(도)


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def amcl_callback(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    latest_pose.update(x=p.x, y=p.y, yaw=quat_to_yaw_deg(q))


# [설명] 도착 오차를 재는 함수. 이 값이 CSV 의 actual_x · actual_y · pos_err_m 이 되고, 서버로 보내는 위치도 이 값이다.
def get_pose(tf_buffer):
    """실제 위치: TF map→base_link. TF 를 못 읽으면 마지막 amcl_pose 사용. 반환: (pose dict, 'tf' | 'amcl' | None)"""
    try:
        # Time() 은 '가장 최근 변환' 을 뜻한다.
        t = tf_buffer.lookup_transform('map', 'base_link', Time())
        tr = t.transform.translation
        return {'x': tr.x, 'y': tr.y, 'yaw': quat_to_yaw_deg(t.transform.rotation)}, 'tf'
    except TransformException:
        return (dict(latest_pose), 'amcl') if latest_pose else ({}, None)


# [설명] time.sleep() 대신 쓴다. sleep 은 콜백을 멈추지만, 이 함수는 기다리는 동안에도 amcl_pose · TF · dock_status 를 받는다.
def spin_for(navigator, sec):
    """sec 초 동안 콜백 처리 (amcl_pose · dock_status 갱신용)."""
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(navigator, timeout_sec=0.1)


CSV_FIELDS = ['list_id', 'goal', 'attempt', 'result', 'elapsed_s', 'goal_x', 'goal_y', 'goal_yaw_deg',
              'actual_x', 'actual_y', 'actual_yaw_deg', 'pos_err_m', 'yaw_err_deg', 'pose_src',
              'capture_s', 'image_file', 'published']


def make_row(list_id, goal, attempt, result, elapsed, actual, pose_src):
    gx, gy = goal['position']
    gyaw = float(goal['yaw_deg'])
    row = {'list_id': list_id, 'goal': goal['name'], 'attempt': attempt,
           'result': getattr(result, 'name', str(result)),
           'elapsed_s': round(elapsed, 2), 'goal_x': gx, 'goal_y': gy, 'goal_yaw_deg': round(gyaw, 1),
           'actual_x': None, 'actual_y': None, 'actual_yaw_deg': None, 'pos_err_m': None, 'yaw_err_deg': None,
           'pose_src': pose_src, 'capture_s': None, 'image_file': None, 'published': False}
    if actual:
        ax, ay, ayaw = actual['x'], actual['y'], actual['yaw']
        # 방향 오차는 -180 ~ 180도로 접는다 (예: 목표 350도, 실제 10도 → 오차 20도. 그냥 빼면 -340도가 된다).
        row.update(actual_x=round(ax, 3), actual_y=round(ay, 3), actual_yaw_deg=round(ayaw, 1),
                   pos_err_m=round(math.hypot(ax - gx, ay - gy), 3),
                   yaw_err_deg=round((ayaw - gyaw + 180) % 360 - 180, 1))
    return row


# ===== 출발 준비 (a_mission.py 의 기능 1 과 같음) ==========================
def prepare_start(navigator):
    """출발 준비 (3_1_a · 3_1_f 순서): undock 상태면 먼저 도킹 → undock → 초기 위치 설정.

    반환: 기록용 dict · 도킹에 실패하면 None
    """
    info = {'docked_before': navigator.getDockedStatus(), 'dock_s': 0.0}
    # [설명] undock 상태로 켜졌으면 로봇이 어디 있는지 모른다 → 먼저 도킹해서 위치를 '도킹 자리' 로 확정한다.
    if not info['docked_before']:
        navigator.info('Docking before intialising pose')  # 3_1_a
        t0 = time.time()
        navigator.dock()
        # dock_status 는 토픽으로 조금 늦게 온다. 1초 동안 콜백을 처리한 뒤에 읽어야 맞는 값이 나온다.
        spin_for(navigator, 1.0)  # dock_status 갱신 대기
        info['dock_s'] = round(time.time() - t0, 2)
        if not navigator.getDockedStatus():
            navigator.error('Robot failed to dock')  # 3_1_f
            return None
    t0 = time.time()
    navigator.undock()
    spin_for(navigator, 1.0)
    info['undock_s'] = round(time.time() - t0, 2)
    info['docked_after'] = navigator.getDockedStatus()
    # [설명] RViz 의 2D Pose Estimate 를 코드가 대신한다 (initialpose 발행 → amcl 이 이 위치에서 추정을 시작).
    navigator.setInitialPose(navigator.getPoseStamped(INITIAL_POSITION, INITIAL_DIRECTION))
    return info


def save_start_metrics(navigator, info):
    """출발 준비 검증 수치 저장 (2_4_e 의 csv 저장 방식)."""
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_start_metrics.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(info))
        writer.writeheader()
        writer.writerow(info)
    navigator.info(f'출발 준비 결과: {info}')
    navigator.info(f'저장: {path}')


# ===== 캡처 노드 (day2/2_1_d_capture_image.py · a_mission.py 의 기능 2 와 같음) =====
class ImageCaptureNode(Node):
    """day2/2_1_d_capture_image.py 의 ImageCaptureNode (바꾼 곳: 토픽 이름, 큐 10 → 1)."""

    def __init__(self, save_directory, file_prefix):
        super().__init__('image_capture_node')
        # [주의] 이 구독은 임무 내내 열려 있어 원본 영상이 계속 PC 로 들어온다 (11/19 측정 약 2.5 MB/s).
        #        v2 · v3 실행에서 위치 변환이 늦어진 원인 후보라서, 사진을 찍을 때만 구독하는 방식을 검토 중이다.
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,              # 2_1_d: '/robot0/oakd/rgb/preview/image_raw'
            self.listener_callback,
            1)                         # 2_1_d: 10 → 1 (가장 최근 프레임 1장만 남김)
        self.bridge = CvBridge()
        self.frame = None
        self.save_directory = save_directory
        self.file_prefix = f"{file_prefix}_"
        self.image_count = 0
        os.makedirs(self.save_directory, exist_ok=True)

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def capture_image(node):
    """2_1_d main 의 spin_once 반복 + 'c' 키 저장 부분. 키 대신 도착 후 1번 호출."""
    # [설명] 도착 전에 받아 둔 프레임을 버린다. 정지한 뒤 새로 들어온 프레임만 써야 이동 중에 찍힌 사진이 섞이지 않는다.
    node.frame = None                                   # 추가: 새로 받은 프레임만 쓰기
    t0 = time.time()
    # 캡처 노드는 평소에 spin 하지 않는다. 여기서 spin_once 를 돌릴 때만 프레임을 받는다 (최대 CAPTURE_TIMEOUT 초).
    while node.frame is None and time.time() - t0 < CAPTURE_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)          # 2_1_d
    if node.frame is None:
        return None, None
    frame = cv2.resize(node.frame, (IMAGE_SIZE, IMAGE_SIZE))   # 추가: 352x352
    file_name = os.path.join(
        node.save_directory,
        f"{node.file_prefix}img_{node.image_count}.jpg")       # 2_1_d
    cv2.imwrite(file_name, frame)                              # 2_1_d
    print(f"Image saved: {file_name}")                         # 2_1_d
    node.image_count += 1                                      # 2_1_d
    return frame, file_name


class MissionPublisher(Node):
    """day2/2_0_a(사진 발행) + 2_0_c(ID·시각 발행). 타이머 대신 도착할 때 publish_photo() 호출."""

    def __init__(self):
        super().__init__('a_remission_publisher')
        self.image_publisher = self.create_publisher(CompressedImage, IMAGE_TOPIC, 10)   # 2_0_a (설계도: JPEG 압축)
        self.data_publisher = self.create_publisher(String, DATA_TOPIC, 10)              # 2_0_c
        self.bridge = CvBridge()                                                         # 2_0_a
        self.id_counter = 1                                                              # 2_0_c

    # [설명] 사진(detection_image)과 데이터(capture_info)는 토픽이 달라 서버에 따로 도착한다.
    #        그래서 같은 번호를 양쪽에 넣는다: 사진은 header.frame_id, 데이터는 JSON 의 'id'.
    #        capture_info 는 JSON 문자열이다: id · robot_id · time · sector · x · y · yaw_deg · mode · stamp_sec · stamp_nanosec
    #        x · y · yaw_deg 는 목표 좌표가 아니라 TF 로 잰 실제 정지 위치다 (m, m, 도).
    # [주의] 칸 이름을 바꾸면 dummy_receiver.py · dummy_remission_server.py 와 서버 쪽도 같이 바꿔야 한다.
    def publish_photo(self, frame, sector, mode, x, y, yaw):
        """mode: 'patrol'(순찰) | 'recheck'(재순찰). 사진과 데이터는 같은 번호(id)로 짝을 맞춤."""
        current_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())  # 2_0_c
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')   # 2_0_a (압축해서 보냄)
        msg.header.stamp = self.get_clock().now().to_msg()                    # 추가: 촬영 시각
        msg.header.frame_id = str(self.id_counter)                            # 추가: 데이터와 같은 번호
        self.image_publisher.publish(msg)                                     # 2_0_a
        info = {                                         # 추가: JSON 으로 보낼 내용 (받는 쪽이 칸 이름으로 꺼냄)
            'id': self.id_counter,                       # 사진 header.frame_id 와 같은 번호
            'robot_id': 'robot_a',
            'time': current_time,
            'sector': sector,
            'x': x, 'y': y, 'yaw_deg': yaw,              # TF 로 잰 실제 정지 위치 (m, m, 도)
            'mode': mode,                                # 'patrol' | 'recheck'
            'stamp_sec': msg.header.stamp.sec,           # 사진 header.stamp 와 같은 값 (시각으로도 짝 맞춤 가능)
            'stamp_nanosec': msg.header.stamp.nanosec,
        }
        message = json.dumps(info, ensure_ascii=False)   # 추가: dict → JSON 문자열
        self.data_publisher.publish(String(data=message))                     # 2_0_c
        self.get_logger().info(f'Published data: {message}')                 # 2_0_c
        self.id_counter += 1                                                  # 2_0_c


# ===== 재순찰 목록 요청 (서비스 remission_list) ============================
# [설명] 서버가 꺼져 있거나 답이 없어도 빈 목록 ('', []) 을 돌려준다 → 로봇은 오류로 죽지 않고 계속 기다린다.
def request_list(navigator, client):
    """서버에 재순찰 목록 요청. 반환: (list_id, [지점 dict, ...]) · 서버가 없거나 응답이 없으면 ('', [])"""
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
        navigator.warn('remission_list 서비스가 없음 (서버가 꺼져 있음?)')
        return '', []
    future = client.call_async(RemissionList.Request(robot_id=ROBOT_ID))
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=SERVICE_TIMEOUT)
    response = future.result() if future.done() else None
    if response is None:
        navigator.warn('remission_list 응답이 없음')
        return '', []
    # [주의] 단위: ZonePose.yaw 는 라디안, 이 파일의 yaw_deg 는 도 → 여기서 한 번만 변환한다.
    #        서버가 도 단위로 넣으면 로봇이 엉뚱한 방향을 본다.
    goals = [{'name': z.zone, 'position': [z.x, z.y], 'yaw_deg': math.degrees(z.yaw)} for z in response.zones]
    return response.list_id, goals


def main():
    # TB4 TF 는 /robot5/tf 로 나옴 → tf 리스너가 네임스페이스 tf 를 받도록 리매핑 (실행 명령은 그대로)
    # 이 리매핑이 없으면 TF 를 받지 못해 get_pose() 가 매번 amcl_pose 로 떨어진다 (CSV 의 pose_src 가 amcl 로 찍힌다).
    rclpy.init(args=sys.argv + ['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    navigator = TurtleBot4Navigator()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)  # navigator 를 spin 할 때 TF 수신

    # amcl_pose 는 '마지막 값을 보관해 두는' 방식(TRANSIENT_LOCAL)으로 발행된다.
    # 같은 QoS 로 구독하면 이 노드가 늦게 켜져도 마지막 위치를 바로 받는다.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    navigator.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', amcl_callback, qos)
    client = navigator.create_client(RemissionList, 'remission_list')

    # 캡처 노드(2_1_d) · 서버 발행 노드(2_0_a + 2_0_c)
    capture_node = ImageCaptureNode(IMAGE_DIR, time.strftime('%Y%m%d_%H%M%S'))
    publisher = MissionPublisher()

    # 검증 수치 CSV: 시작할 때 만들고 시도마다 한 줄씩 바로 저장 (2_4_e 의 csv 저장 방식)
    # 시도마다 바로 flush 하므로, 막혀서 Ctrl+C 로 끊은 실행도 그때까지의 줄이 남는다.
    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_remission_metrics.csv')
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()
    csv_file.flush()
    navigator.info(f'기록 파일: {csv_path}')

    # [설명] 키는 '목록ID/구역' 이다. 같은 목록을 다시 받아도 다시 가지 않고,
    #        목록 ID 가 바뀌면 같은 구역도 새 재확인 요청으로 보고 다시 간다.
    finished = set()   # 끝낸 지점 ('목록 ID/구역') - 도착했거나 3회 실패해 건너뛴 곳
    rows = []

    while True:
        # 1. 받은 데이터 있음? (설계도 ②: 없으면 그 자리(도킹)에서 대기하며 다시 확인)
        t0, last_log = time.time(), 0.0
        list_id, goals = request_list(navigator, client)
        while not [g for g in goals if f"{list_id}/{g['name']}" not in finished]:
            if time.time() - last_log > 15.0:
                navigator.info('재순찰 목록 대기 중... (서비스 remission_list)')
                last_log = time.time()
            spin_for(navigator, POLL_EVERY)
            list_id, goals = request_list(navigator, client)
        wait_s = time.time() - t0
        navigator.info(f"목록 수신 ({wait_s:.1f}초 대기): list_id={list_id} · {[g['name'] for g in goals]}")

        # 2. 출발 준비: RViz 없이 도킹 자리에서 출발 (undock 상태면 먼저 도킹)
        # [설명] 목록을 받을 때마다 도킹 자리에서 위치를 다시 잡는다 → 여러 번 돌아도 위치 오차가 쌓이지 않는다.
        #        대신 목록마다 도킹 · undock 시간이 든다.
        info = prepare_start(navigator)
        if info is None:
            break
        initial_pose = navigator.getPoseStamped(INITIAL_POSITION, INITIAL_DIRECTION)

        # 3. amcl 위치 대기
        # amcl 이 켜지기 전에 보낸 초기 위치는 버려진다 → 반영될 때까지 5초마다 다시 보낸다.
        # LOCALIZE_TIMEOUT(30초)이 지나도 못 받으면 localization 이나 scan 문제로 보고 중단한다.
        t0, last_send = time.time(), time.time()
        while not (navigator.initial_pose_received and latest_pose) and time.time() - t0 < LOCALIZE_TIMEOUT:
            rclpy.spin_once(navigator, timeout_sec=0.5)
            if time.time() - last_send > 5.0:  # 응답이 없으면 5초마다 다시 보냄
                navigator.setInitialPose(initial_pose)
                last_send = time.time()
        localize_s = time.time() - t0
        if not navigator.initial_pose_received:
            navigator.error(f'{LOCALIZE_TIMEOUT:.0f}초 안에 amcl 위치를 받지 못함. localization · /scan 확인 후 다시 실행')
            break
        info.update(localize_s=round(localize_s, 1), list_id=list_id, list_wait_s=round(wait_s, 2))
        save_start_metrics(navigator, info)

        # 4. Nav2 대기
        # [주의] Nav2 는 켜진 뒤 map→base_link 변환을 60초만 기다린다. 초기 위치가 그보다 늦으면 Nav2 기동이 실패한다
        #        (11/19 에 여러 번 발생 → nav2.yaml 의 initial_transform_timeout 을 늘려 해결).
        navigator.info('Nav2 활성화 대기 중... (Nav2 터미널에 Managed nodes are active 가 나올 때까지)')
        navigator.waitUntilNav2Active()

        # 5. 목록의 지점마다 이동 → 캡처 → 송신 (mode=recheck). 다 돌면 30초 동안 새 목록을 기다림
        while True:
            todo = [g for g in goals if f"{list_id}/{g['name']}" not in finished]
            # [TODO] 설계도 ②에는 지점마다 배터리 20% 확인이 있지만 이 파일에는 아직 없다 (a_mission.py 의 기능 4 를 옮겨야 한다).
            for goal in todo:
                pose = navigator.getPoseStamped([float(v) for v in goal['position']], goal['yaw_deg'])  # 정수 좌표도 실수로 변환
                for attempt in range(1, MAX_TRIES + 1):
                    navigator.info(f"{goal['name']} 이동 시작 ({attempt}/{MAX_TRIES})")
                    t0 = time.time()
                    navigator.startToPose(pose)
                    result = navigator.getResult()
                    elapsed = time.time() - t0
                    spin_for(navigator, 1.0)  # 정지 후 TF · amcl_pose 갱신 대기
                    actual, pose_src = get_pose(tf_buffer)
                    row = make_row(list_id, goal, attempt, result, elapsed, actual, pose_src)
                    navigator.info(f"  {row['result']} · {row['elapsed_s']}초 · 위치 오차 {row['pos_err_m']} m · "
                                   f"방향 오차 {row['yaw_err_deg']}도 ({pose_src})")
                    # [설명] 도착 판정이 난 지점에서만 찍는다. 실패한 위치의 사진이 서버로 가면 다른 섹터 사진으로 잘못 쓰일 수 있다.
                    if result == TaskResult.SUCCEEDED:
                        # 도착 → 사진 1장 캡처 (2_1_d) → 서버로 발행 (2_0_a + 2_0_c) · 재확인 결과
                        t_cap = time.time()
                        frame, file_name = capture_image(capture_node)
                        row['capture_s'] = round(time.time() - t_cap, 2)
                        row['image_file'] = file_name
                        if frame is not None:
                            # 'recheck' = 재확인 사진. 서버가 이 값으로 첫 조사('patrol')와 구분하고,
                            # dummy_remission_server.py 도 이 값을 보고 목록에서 구역을 뺀다.
                            publisher.publish_photo(frame, goal['name'], 'recheck',
                                                    row['actual_x'], row['actual_y'], row['actual_yaw_deg'])
                            row['published'] = True
                        else:
                            navigator.warn(f"{goal['name']} 사진을 받지 못함 ({CAPTURE_TIMEOUT}초)")
                    rows.append(row)
                    writer.writerow(row)
                    csv_file.flush()
                    if result == TaskResult.SUCCEEDED:
                        break
                    # 실패 원인이 costmap 에 남은 장애물 흔적일 수 있어서 비우고 다시 시도한다.
                    navigator.clearAllCostmaps()
                    if attempt < MAX_TRIES:
                        navigator.info(f'  {RETRY_WAIT:.0f}초 기다린 뒤 다시 시도')
                        spin_for(navigator, RETRY_WAIT)
                finished.add(f"{list_id}/{goal['name']}")  # 도착했거나 3회 실패 → 다시 가지 않음

            # 남은 좌표? (설계도 ②: 없으면 30초 동안 자리에서 수신 대기)
            # [설명] 바로 복귀하지 않고 30초 기다린다 (설계도 ②).
            #        그 사이에 새 목록이 오면 도킹 자리까지 왕복하지 않고 이어서 간다.
            navigator.info(f'목록 완료 → {WAIT_FOR_LIST:.0f}초 동안 새 목록 대기')
            t0, got_new = time.time(), False
            while time.time() - t0 < WAIT_FOR_LIST:
                spin_for(navigator, POLL_EVERY)
                list_id, goals = request_list(navigator, client)
                if [g for g in goals if f"{list_id}/{g['name']}" not in finished]:
                    navigator.info(f"새 목록 수신: list_id={list_id} · {[g['name'] for g in goals]}")
                    got_new = True
                    break
            if not got_new:
                break

        # 6. docking station 이동 → 도킹 (도크 앞으로 간 뒤 도킹, 3_1_f 순서)
        navigator.info('새 목록 없음 → docking station 으로 이동')
        # dock() 은 도크 근처에서만 성공한다 → 먼저 도킹 자리 앞(초기 위치)까지 Nav2 로 이동한다.
        navigator.startToPose(initial_pose)
        navigator.dock()

        # 7. 요약 (CSV 는 위에서 시도마다 저장 완료)
        arrived = [r for r in rows if r['result'] == 'SUCCEEDED']
        shots = [r for r in rows if r['published']]
        errs = [r['pos_err_m'] for r in arrived if r['pos_err_m'] is not None]
        navigator.info(f"===== 결과: 도착 {len(arrived)}/{len(finished)} · 사진 발행 {len(shots)}/{len(arrived)} · "
                       f"평균 위치 오차 {round(sum(errs) / len(errs), 3) if errs else None} m =====")
        navigator.info(f'저장: {csv_path}')
        if RUN_ONCE:
            break

    # 노드 정리 (2_1_d finally 부분)
    csv_file.close()
    capture_node.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
