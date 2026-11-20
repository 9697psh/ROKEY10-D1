# /farm/sector_complete : PC2 -> PC1, server_bridge.py (문서 완료 로그 항목)

string COMPLETION_AUTO="AUTO"
string COMPLETION_MANUAL_OVERRIDE="MANUAL_OVERRIDE"

std_msgs/Header header          # stamp = 완료 확정 시각
string event_id                 # 중복 기록 방지
string mission_id
string task_id
string sector_id
uint32 revision
string command_id               # 강제 완료 명령 ID (AUTO 면 빈 값)
string completion_mode          # AUTO / MANUAL_OVERRIDE
uint32 remaining_count          # 로봇 마지막 판단 잔여 수량 (0 으로 고치지 않음)
string last_state               # 진행 / HOLD 등 마지막 판단 상태
string unable_reason            # 가림 등 판단 불가 사유
string reason                   # 강제 완료 사유
string worker_id                # 작업자 ID (있을 때)
string ref_photo_id
string final_photo_id
builtin_interfaces/Time decision_stamp   # 판단 시각
builtin_interfaces/Time request_stamp    # 강제 완료 요청 시각
