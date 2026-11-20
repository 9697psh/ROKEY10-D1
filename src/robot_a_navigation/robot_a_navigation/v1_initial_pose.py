#!/usr/bin/env python3
"""[테스트 버전 1] 로봇 A 작업 관리 (a_mission) - RViz 없이 초기 좌표로 출발.

들어 있는 기능: ① 초기 좌표 출발
  ① undock 상태면 먼저 도킹 → undock → INITIAL_POSITION 을 초기 위치로 설정 → 모든 좌표로 이동
참고: day3/3_1_a·3_1_f (도킹 → undock → 초기 위치 순서), 3_1_c (순회·dock), 3_1_d (좌표 없으면 종료), day2/2_4_e (CSV)

실행 순서: localization → nav2 → 이 파일 (RViz 2D Pose Estimate 는 필요 없음)
    ros2 run robot_a_navigation v1_initial_pose --ros-args -r __ns:=/robot5
결과 (~/farmmate_ws/logs/a_mission/):
  <시각>_start_metrics.csv  도킹 상태 전후, 도킹·undock 시간, 위치 확인 시간, 초기 위치 오차
  <시각>_nav_metrics.csv    지점·시도별 시간, 위치/방향 오차
"""

# [설명] 테스트 버전 1: [기능 1] RViz 없이 출발 + 지점 순회만 들어 있다. v2 가 여기에 캡처 · 송신을 더한다.
# [설명] 읽는 순서: 설정값(좌표 · 제한 시간) → 위치 측정 함수 → [기능 1] 출발 준비 → main()
# [설명] 주석 끝의 '2_1_d', '3_1_f' 같은 표시는 그 줄을 가져온 강의 코드 파일이다 (to_students day2 · day3).

import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# [설명] 로봇은 아래 목록을 위에서부터 순서대로 방문한다.
#        방향 값: NORTH 0도 · WEST 90도 · SOUTH 180도 · EAST 270도 (CSV 의 goal_yaw_deg 에 이 숫자로 남는다)
#        B_4 와 C_1 은 좌표가 같고 방향만 반대다 (한 자리에서 반대쪽을 한 번 더 찍는다).
# [주의] 좌표는 반드시 실수로 쓴다 ([0, 0] 은 안 됨 → [0.0, 0.0]). 정수를 그대로 보내면 Python 오류 메시지 없이
#        프로세스가 죽는다 (11/19 v4 실행에서 확인. main 의 getPoseStamped 줄에서 float 로 한 번 더 막는다).
# ===== 좌표 (지도 좌표 [x, y], 방향은 도 단위 숫자 또는 TurtleBot4Directions) =====
GOALS = [{'name': 'A_1', 'position': [-0.57, -0.15], 'yaw_deg' : TurtleBot4Directions.SOUTH},
         {'name': 'A_2', 'position': [-1.89, 0.037], 'yaw_deg' : TurtleBot4Directions.NORTH},
         {'name': 'B_1', 'position': [-0.393, 3.5], 'yaw_deg' : TurtleBot4Directions.WEST},
         {'name': 'B_2', 'position': [-1.01, 3.5], 'yaw_deg' : TurtleBot4Directions.WEST},
         {'name': 'B_3', 'position': [-1.14, 4.7], 'yaw_deg' : TurtleBot4Directions.EAST},
         {'name': 'B_4', 'position': [-0.228, 4.7], 'yaw_deg' : TurtleBot4Directions.EAST},
         {'name': 'C_1', 'position': [-0.228, 4.7], 'yaw_deg' : TurtleBot4Directions.WEST},
         {'name': 'C_2', 'position': [-1.14, 4.7], 'yaw_deg' : TurtleBot4Directions.WEST},
         {'name': 'C_3', 'position': [-0.98, 5.9], 'yaw_deg' : TurtleBot4Directions.EAST},
         {'name': 'C_4', 'position': [-0.06, 5.9], 'yaw_deg' : TurtleBot4Directions.EAST},
         {'name': 'Origin', 'position': [0.0, 0.0], 'yaw_deg' : TurtleBot4Directions.NORTH}
]

# [설명] 출발 위치를 코드에 고정한다. 항상 도킹 자리에서 출발하므로 RViz 에서 위치를 찍지 않아도 된다.
#        맵을 새로 만들면 원점이 달라질 수 있으니 이 값도 다시 확인한다.
INITIAL_POSITION = [0.0, 0.0]                  # undock 직후 위치 (항상 도킹 자리에서 출발)
INITIAL_DIRECTION = TurtleBot4Directions.SOUTH
# [TODO] 설계도 ①은 3회 실패 시 '이전 좌표 / 도킹 위치로 이동' 이지만, 지금은 그 지점을 건너뛰고 다음으로 간다.
MAX_TRIES = 3            # 지점마다 최대 시도 (흐름도 ①: 3회 실패 시 건너뜀)
# [설명] 11/19 v2 실행에서 실패 직후 바로 재시도해 짧은 통신 지연에 3회를 모두 써 버렸다 → 재시도 전에 쉰다.
RETRY_WAIT = 3.0         # 실패 후 다시 시도하기 전 대기(초) - 통신·위치 지연이 풀릴 시간
LOCALIZE_TIMEOUT = 30.0  # amcl 위치를 기다리는 최대 시간(초)
DOCK_AT_END = True       # 끝나면 도킹
LOG_DIR = os.path.expanduser('~/farmmate_ws/logs/a_mission')

# [설명] amcl_callback 이 채우는 값. 초기 위치가 반영됐는지 확인할 때와, TF 를 못 읽을 때의 예비 값으로 쓴다.
latest_pose = {}  # 마지막 amcl_pose: x, y, yaw(도)


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def amcl_callback(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    latest_pose.update(x=p.x, y=p.y, yaw=quat_to_yaw_deg(q))


# [설명] 도착 오차를 재는 함수. 이 값이 CSV 의 actual_x · actual_y · pos_err_m 이 된다.
def get_pose(tf_buffer):
    """실제 위치: TF map→base_link. amcl_pose 는 0.25 m 움직여야 갱신돼서 정지 위치와 다를 수 있음.
    TF 를 못 읽으면 마지막 amcl_pose 사용. 반환: (pose dict, 'tf' | 'amcl' | None)"""
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


CSV_FIELDS = ['goal', 'attempt', 'result', 'elapsed_s', 'goal_x', 'goal_y', 'goal_yaw_deg',
              'actual_x', 'actual_y', 'actual_yaw_deg', 'pos_err_m', 'yaw_err_deg', 'pose_src',
              'start', 'localize_s']


def make_row(goal, attempt, result, elapsed, start, localize_s, actual, pose_src):
    gx, gy = goal['position']
    gyaw = float(goal['yaw_deg'])
    row = {'goal': goal['name'], 'attempt': attempt, 'result': getattr(result, 'name', str(result)),
           'elapsed_s': round(elapsed, 2), 'goal_x': gx, 'goal_y': gy, 'goal_yaw_deg': gyaw,
           'actual_x': None, 'actual_y': None, 'actual_yaw_deg': None, 'pos_err_m': None, 'yaw_err_deg': None,
           'pose_src': pose_src, 'start': start, 'localize_s': round(localize_s, 1)}
    if actual:
        ax, ay, ayaw = actual['x'], actual['y'], actual['yaw']
        # 방향 오차는 -180 ~ 180도로 접는다 (예: 목표 350도, 실제 10도 → 오차 20도. 그냥 빼면 -340도가 된다).
        row.update(actual_x=round(ax, 3), actual_y=round(ay, 3), actual_yaw_deg=round(ayaw, 1),
                   pos_err_m=round(math.hypot(ax - gx, ay - gy), 3),
                   yaw_err_deg=round((ayaw - gyaw + 180) % 360 - 180, 1))
    return row


# ===== [기능 1] RViz 없이 초기 좌표로 출발 ===============================
def prepare_start(navigator):
    """출발 준비 (3_1_a · 3_1_f 순서): undock 상태면 먼저 도킹 → undock → 초기 위치 설정.

    도킹 자리에서 출발해야 INITIAL_POSITION 을 초기 위치로 믿을 수 있어서 RViz 2D Pose Estimate 가 필요 없음.
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


def main():
    # TB4 TF 는 /robot5/tf 로 나옴 → tf 리스너가 네임스페이스 tf 를 받도록 리매핑 (실행 명령은 그대로)
    # 이 리매핑이 없으면 TF 를 받지 못해 get_pose() 가 매번 amcl_pose 로 떨어진다 (CSV 의 pose_src 가 amcl 로 찍힌다).
    rclpy.init(args=sys.argv + ['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    navigator = TurtleBot4Navigator()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)  # navigator 를 spin 할 때 TF 수신

    # 좌표가 없으면 로봇을 움직이지 않고 종료 (3_1_d_create_path.py)
    if len(GOALS) == 0:
        navigator.error('No poses were given, exiting.')
        rclpy.shutdown()
        return

    # amcl_pose 는 '마지막 값을 보관해 두는' 방식(TRANSIENT_LOCAL)으로 발행된다.
    # 같은 QoS 로 구독하면 이 노드가 늦게 켜져도 마지막 위치를 바로 받는다.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    navigator.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', amcl_callback, qos)

    # 1. 출발 준비: RViz 없이 도킹 자리에서 출발 (undock 상태면 먼저 도킹)
    info = prepare_start(navigator)
    if info is None:
        rclpy.shutdown()
        return
    initial_pose = navigator.getPoseStamped(INITIAL_POSITION, INITIAL_DIRECTION)

    # 2. amcl 위치 대기 (BasicNavigator 가 amcl_pose 를 받으면 initial_pose_received = True)
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
        rclpy.shutdown()
        return
    navigator.info(f'위치 확인 ({localize_s:.1f}초): {latest_pose}')

    # 출발 준비 검증 수치: 초기 위치가 amcl 에 반영됐는지 (지정한 초기 위치와 amcl 위치의 차이)
    info.update(localize_s=round(localize_s, 1), amcl_x=round(latest_pose['x'], 3), amcl_y=round(latest_pose['y'], 3),
                init_err_m=round(math.hypot(latest_pose['x'] - INITIAL_POSITION[0],
                                            latest_pose['y'] - INITIAL_POSITION[1]), 3))
    save_start_metrics(navigator, info)

    # 3. Nav2 대기 (초기 위치는 위에서 확인했으므로 라이브러리의 초기 위치 반복은 건너뜀)
    # [주의] Nav2 는 켜진 뒤 map→base_link 변환을 60초만 기다린다. 초기 위치가 그보다 늦으면 Nav2 기동이 실패한다
    #        (11/19 에 여러 번 발생 → nav2.yaml 의 initial_transform_timeout 을 늘려 해결).
    navigator.info('Nav2 활성화 대기 중... (Nav2 터미널에 Managed nodes are active 가 나올 때까지)')
    navigator.waitUntilNav2Active()

    # 4. 지점마다 이동 (3_1_c 순회, 지점마다 startToPose, 최대 MAX_TRIES 번)
    start = 'docked' if info['docked_before'] else 'dock_first'
    rows, t_run = [], time.time()

    # 검증 수치 CSV: 시작할 때 만들고 시도마다 한 줄씩 바로 저장 (2_4_e 의 csv 저장 방식)
    # 시도마다 바로 flush 하므로, 막혀서 Ctrl+C 로 끊은 실행도 그때까지의 줄이 남는다.
    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, time.strftime('%Y%m%d_%H%M%S') + '_nav_metrics.csv')
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()
    csv_file.flush()
    navigator.info(f'기록 파일: {csv_path}')

    for goal in GOALS:
        pose = navigator.getPoseStamped([float(v) for v in goal['position']], goal['yaw_deg'])  # 정수 좌표도 실수로 변환
        for attempt in range(1, MAX_TRIES + 1):
            navigator.info(f"{goal['name']} 이동 시작 ({attempt}/{MAX_TRIES})")
            t0 = time.time()
            navigator.startToPose(pose)
            result = navigator.getResult()
            elapsed = time.time() - t0
            spin_for(navigator, 1.0)  # 정지 후 TF · amcl_pose 갱신 대기
            actual, pose_src = get_pose(tf_buffer)
            row = make_row(goal, attempt, result, elapsed, start, localize_s, actual, pose_src)
            navigator.info(f"  {row['result']} · {row['elapsed_s']}초 · 위치 오차 {row['pos_err_m']} m · "
                           f"방향 오차 {row['yaw_err_deg']}도 ({pose_src})")
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

    # 5. 요약 (CSV 는 위에서 시도마다 저장 완료)
    csv_file.close()
    arrived = [r for r in rows if r['result'] == 'SUCCEEDED']
    errs = [r['pos_err_m'] for r in arrived if r['pos_err_m'] is not None]
    navigator.info(f"===== 결과: 도착 {len(arrived)}/{len(GOALS)} · 총 {time.time() - t_run:.1f}초 · "
                   f"평균 위치 오차 {round(sum(errs) / len(errs), 3) if errs else None} m =====")
    navigator.info(f'저장: {csv_path}')

    # 6. 끝나면 도킹 (3_1_c_follow_waypoints.py)
    if DOCK_AT_END:
        navigator.dock()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
