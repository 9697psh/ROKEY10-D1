# farmmate_ws

FarmMate 프로젝트 로봇 A(TurtleBot4) 작업공간입니다. ROS 2 Jazzy · Nav2 기준.

## 구성

| 경로 | 내용 |
|---|---|
| `src/farm_interfaces` | 직접 만든 메시지 정의 (msg 7 · srv 1 · action 1) |
| `src/robot_a_bringup` | Nav2 설정(`config/nav2.yaml`), SLAM 런치, 지도(`maps/`) |
| `src/robot_a_mapping` | 지도 저장 노드 |
| `src/robot_a_navigation` | 로봇 A 임무 코드와 시험 버전 · 더미 |
| `tools/` | 메시지 왕복 시험 스크립트 |
| `logs/` | 실행 기록(CSV), 설정 · 코드 백업 |

`build/`, `install/`, `log/` 는 올리지 않았습니다. 아래 순서로 빌드하면 생깁니다.

## robot_a_navigation 실행 파일

| 이름 | 내용 |
|---|---|
| `a_mission` | 통합본: 서버 명령 → 출발 → 좌표 이동 → 캡처 · 송신 → 배터리 복귀 → 도킹 |
| `a_remission` | 재순찰: 서버 목록 요청 → 이동 → 캡처 · 송신 |
| `v1_initial_pose` ~ `v4_battery` | 기능을 하나씩 더한 시험 버전 |
| `dummy_server`, `dummy_receiver`, `dummy_battery`, `dummy_remission_server` | 서버 · 배터리 역할을 대신하는 더미 |

## 빌드

TurtleBot4 패키지(`turtlebot4_navigation`)가 설치된 ROS 2 Jazzy 환경이 필요합니다.

```bash
git clone <이 저장소 주소> ~/farmmate_ws
cd ~/farmmate_ws
colcon build
source install/setup.bash
```

## 실행 (로봇 네임스페이스 /robot5 예)

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot5 map:=$HOME/farmmate_ws/src/robot_a_bringup/maps/farm_map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot5 params_file:=$HOME/farmmate_ws/src/robot_a_bringup/config/nav2.yaml
ros2 run robot_a_navigation a_mission --ros-args -r __ns:=/robot5
ros2 run robot_a_navigation dummy_server --ros-args -r __ns:=/robot5
```

각 파일 맨 위 설명에 실행 순서와 짝이 되는 더미가 적혀 있습니다.
