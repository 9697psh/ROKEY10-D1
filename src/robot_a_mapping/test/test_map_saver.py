import math

import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid

from robot_a_mapping.map_saver_node import (PX_FREE, PX_OCCUPIED, PX_UNKNOWN, grid_to_pgm,
                                            yaw_from_quaternion)


def test_grid_to_pgm_trinary_and_flip():
    g = OccupancyGrid()
    g.info.width, g.info.height = 3, 2
    # 아래 행(y=0): 빈칸, 벽, 미탐색 / 위 행(y=1): 중간값(미확정), 0, 100
    g.data = [0, 100, -1, 50, 0, 100]
    img = grid_to_pgm(g, 0.65, 0.25)
    assert img.shape == (2, 3)
    # flipud → 이미지 첫 행이 지도 위 행
    assert img[0].tolist() == [PX_UNKNOWN, PX_FREE, PX_OCCUPIED]
    assert img[1].tolist() == [PX_FREE, PX_OCCUPIED, PX_UNKNOWN]
    assert img.dtype == np.uint8


def test_yaw_from_quaternion():
    q = Quaternion(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
    assert abs(yaw_from_quaternion(q) - math.pi / 2) < 1e-9
