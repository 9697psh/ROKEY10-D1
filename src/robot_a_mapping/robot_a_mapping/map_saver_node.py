"""SLAM 지도 저장 노드 (실행 흐름 ①).

slam_toolbox가 발행하는 /map(OccupancyGrid)을 받아 Nav2 map_server가 읽는
.pgm + .yaml 파일로 저장한다. nav2 map_saver_server 없이 혼자 동작한다.

    ros2 run robot_a_mapping map_saver_node --ros-args -r __ns:=/robot<n>
        Enter      → 지금 지도를 저장
        q + Enter  → 종료

    # 키보드 대신 서비스로 저장
    ros2 service call /robot<n>/map_saver_node/save std_srvs/srv/Trigger

저장 위치 기본값: ~/farmmate_ws/src/robot_a_bringup/maps/farm_map.{yaml,pgm}
같은 이름 파일이 있으면 지우지 않고 farm_map_<시각>.* 으로 옮겨 둔다.
"""

import math
import os
import shutil
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:  # slam_toolbox 파이썬 인터페이스가 없으면 포즈 그래프 저장만 건너뜀
    SerializePoseGraph = None

DEFAULT_MAP_DIR = os.path.expanduser('~/farmmate_ws/src/robot_a_bringup/maps')

# PGM 픽셀 값 (nav2 map_saver trinary 모드와 같음)
PX_OCCUPIED, PX_FREE, PX_UNKNOWN = 0, 254, 205


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_to_pgm(grid: OccupancyGrid, occupied_thresh: float, free_thresh: float) -> np.ndarray:
    """OccupancyGrid(-1, 0~100) → trinary PGM 픽셀 배열. 위아래를 뒤집어 이미지 좌표로 맞춘다."""
    w, h = grid.info.width, grid.info.height
    cells = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    occ_i = int(round(occupied_thresh * 100))
    free_i = int(round(free_thresh * 100))
    img = np.full((h, w), PX_UNKNOWN, dtype=np.uint8)
    known = (cells >= 0) & (cells <= 100)
    img[known & (cells <= free_i)] = PX_FREE
    img[known & (cells >= occ_i)] = PX_OCCUPIED
    return np.flipud(img)


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver_node')
        self.map_dir = os.path.expanduser(self.declare_parameter('map_dir', DEFAULT_MAP_DIR).value)
        self.map_name = self.declare_parameter('map_name', 'farm_map').value
        map_topic = self.declare_parameter('map_topic', 'map').value
        self.occupied_thresh = float(self.declare_parameter('occupied_thresh', 0.65).value)
        self.free_thresh = float(self.declare_parameter('free_thresh', 0.25).value)
        self.serialize = bool(self.declare_parameter('serialize_posegraph', True).value)
        self.keyboard = bool(self.declare_parameter('keyboard', True).value)
        status_period = float(self.declare_parameter('status_period', 5.0).value)

        self._lock = threading.Lock()
        self._map = None
        self._map_stamp = 0.0
        self._map_count = 0

        # slam_toolbox의 /map은 transient_local(래치) → 같은 QoS로 받아야 마지막 지도를 바로 받는다
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.create_service(Trigger, '~/save', self._on_save_srv)
        self.saved_pub = self.create_publisher(String, '~/saved', 10)
        self.create_timer(status_period, self._log_status)

        self._serialize_cli = None
        if self.serialize and SerializePoseGraph is not None:
            self._serialize_cli = self.create_client(SerializePoseGraph, 'slam_toolbox/serialize_map')

        self.get_logger().info(
            f'지도 대기 중: topic={self.resolve_topic_name(map_topic)} → 저장 위치 '
            f'{os.path.join(self.map_dir, self.map_name)}.yaml')
        if self.keyboard:
            self.get_logger().info('Enter = 지금 지도 저장,  q + Enter = 종료')

    # ---------------- callbacks ----------------
    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            first = self._map is None
            self._map = msg
            self._map_stamp = time.monotonic()
            self._map_count += 1
        if first:
            self.get_logger().info(
                f'첫 지도 수신: {msg.info.width}x{msg.info.height} 셀, '
                f'해상도 {msg.info.resolution:.3f} m')

    def _on_save_srv(self, _req, resp):
        ok, msg = self.save()
        resp.success, resp.message = ok, msg
        return resp

    def _log_status(self):
        with self._lock:
            grid, age = self._map, time.monotonic() - self._map_stamp
        if grid is None:
            self.get_logger().warn('아직 /map을 받지 못했습니다. slam.launch.py와 namespace를 확인하세요.')
            return
        data = np.asarray(grid.data, dtype=np.int16)
        known = np.count_nonzero(data >= 0)
        occ = np.count_nonzero(data >= int(round(self.occupied_thresh * 100)))
        w_m = grid.info.width * grid.info.resolution
        h_m = grid.info.height * grid.info.resolution
        self.get_logger().info(
            f'지도 {w_m:.1f} x {h_m:.1f} m · 탐색된 셀 {known / max(data.size, 1) * 100:.0f}% '
            f'· 벽/장애물 셀 {occ} · 마지막 갱신 {age:.0f}초 전')

    # ---------------- saving ----------------
    def save(self):
        with self._lock:
            grid = self._map
        if grid is None:
            msg = '저장 실패: 아직 /map을 받지 못했습니다.'
            self.get_logger().error(msg)
            return False, msg

        os.makedirs(self.map_dir, exist_ok=True)
        base = os.path.join(self.map_dir, self.map_name)
        yaml_path, pgm_path = base + '.yaml', base + '.pgm'
        backup = self._backup_existing(base)

        img = grid_to_pgm(grid, self.occupied_thresh, self.free_thresh)
        h, w = img.shape
        tmp_pgm = pgm_path + '.tmp'
        with open(tmp_pgm, 'wb') as f:
            f.write(f'P5\n# CREATOR: robot_a_mapping map_saver_node {grid.info.resolution:.3f} m/pix\n'
                    f'{w} {h}\n255\n'.encode('ascii'))
            f.write(img.tobytes())
        os.replace(tmp_pgm, pgm_path)

        o = grid.info.origin
        yaml_text = (
            f'image: {os.path.basename(pgm_path)}\n'
            'mode: trinary\n'
            f'resolution: {grid.info.resolution:.3f}\n'
            f'origin: [{o.position.x:.3f}, {o.position.y:.3f}, {yaw_from_quaternion(o.orientation):.3f}]\n'
            'negate: 0\n'
            f'occupied_thresh: {self.occupied_thresh}\n'
            f'free_thresh: {self.free_thresh}\n'
        )
        tmp_yaml = yaml_path + '.tmp'
        with open(tmp_yaml, 'w') as f:
            f.write(yaml_text)
        os.replace(tmp_yaml, yaml_path)

        msg = f'지도 저장 완료: {yaml_path} ({w}x{h}, {grid.info.resolution:.3f} m/pix)'
        if backup:
            msg += f' · 이전 지도는 {backup}.* 로 보관'
        self.get_logger().info(msg)
        self.get_logger().info(
            f'다음 단계(좌표 설정·Nav2): ros2 launch turtlebot4_navigation localization.launch.py '
            f'namespace:={self.get_namespace()} map:={yaml_path}')
        self.saved_pub.publish(String(data=yaml_path))
        self._serialize_posegraph(base)
        return True, msg

    def _backup_existing(self, base):
        if not (os.path.exists(base + '.yaml') or os.path.exists(base + '.pgm')):
            return None
        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup = f'{base}_{stamp}'
        for ext in ('.yaml', '.pgm'):
            if os.path.exists(base + ext):
                shutil.move(base + ext, backup + ext)
        # 옮긴 yaml이 자기 pgm을 가리키도록 고친다
        if os.path.exists(backup + '.yaml'):
            with open(backup + '.yaml') as f:
                text = f.read()
            text = text.replace(f'image: {os.path.basename(base)}.pgm',
                                f'image: {os.path.basename(backup)}.pgm')
            with open(backup + '.yaml', 'w') as f:
                f.write(text)
        return backup

    def _serialize_posegraph(self, base):
        """slam_toolbox 포즈 그래프(.posegraph/.data)도 저장해 두면 나중에 이어서 매핑할 수 있다."""
        if self._serialize_cli is None:
            return
        if not self._serialize_cli.service_is_ready():
            self.get_logger().info('slam_toolbox/serialize_map 서비스가 없어 포즈 그래프 저장은 건너뜁니다.')
            return
        req = SerializePoseGraph.Request()
        req.filename = base
        fut = self._serialize_cli.call_async(req)
        fut.add_done_callback(lambda f: self.get_logger().info(
            f'포즈 그래프 저장: {base}.posegraph' if f.result() is not None and f.result().result == 0
            else '포즈 그래프 저장 실패 (지도 파일은 정상 저장됨)'))


def _keyboard_loop(node: MapSaverNode):
    while rclpy.ok():
        try:
            line = sys.stdin.readline()
        except (OSError, ValueError):
            return
        if line == '':  # stdin 닫힘
            return
        cmd = line.strip().lower()
        if cmd in ('q', 'quit', 'exit'):
            node.get_logger().info('종료합니다.')
            rclpy.try_shutdown()
            return
        node.save()


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    if node.keyboard and sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=_keyboard_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
