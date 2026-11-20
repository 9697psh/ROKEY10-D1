# /farm/work_targets : PC1 a_mission.py -> PC2 B 작업 관리, server_bridge.py (문서 T05)
# B 는 작업(WorkTarget)마다 TaskAck 1개를 보낸다. A 는 응답 없는 작업만 다시 보낸다.

std_msgs/Header header          # stamp = 발행 시각, frame_id = map
string mission_id
WorkTarget[] tasks
