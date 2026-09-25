# 仅悬空台架模式接收：逻辑轮速，正值均表示推动车辆前进。
# header.stamp必须更新，session_id必须匹配当前会话；不自动使能。
std_msgs/Header header
string session_id
float64 left_rad_s
float64 right_rad_s
