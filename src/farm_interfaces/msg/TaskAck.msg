# /farm/task_ack : PC2 -> PC1 (문서). 수신 응답이며 작업 완료가 아님

std_msgs/Header header
string mission_id
string task_id
uint32 revision
bool accepted
string reason          # 거절 사유 (접수면 빈 값)
