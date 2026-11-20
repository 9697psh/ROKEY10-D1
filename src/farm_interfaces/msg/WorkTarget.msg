# 작업 1개 (문서: 작업별 목표 좌표, 기준 사진, 처리 대상 목록, 버전)

string task_id
uint32 revision                 # 1부터, 재작업 때 +1 (task_id 는 그대로)
string type                     # new / revisit
string sector_id                # 섹터 ID 는 frame_id 가 아니라 별도 필드 (문서)
geometry_msgs/PoseStamped goal  # B 정차 위치 (frame_id = map) -> Nav2 에 그대로 전달
uint32 goal_count               # 처리할 대상 수
TargetObject[] targets
string ref_photo_id
sensor_msgs/CompressedImage ref_photo          # 기준 원본 사진
sensor_msgs/CompressedImage ref_labeled_photo  # 라벨링 사진
