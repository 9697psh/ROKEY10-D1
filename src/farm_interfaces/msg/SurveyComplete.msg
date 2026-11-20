# /farm/survey_complete : PC1 -> PC2, server_bridge.py (이름·필드: 제안)
# 최초 조사 종료 != 전체 작업 종료

std_msgs/Header header
string mission_id
uint32 surveyed_sectors         # 조사한 섹터 수
uint32 tasks_issued             # B 에 보낸 작업 수
uint32 no_target_sectors        # 대상 없음으로 기록한 섹터 수
