# Effective, validated driver limits. Published as a live heartbeat, not latched.
#
# 配信様式: stream (10 Hz, robot_simulator_node の cycle_hz。実機は ypspur_node の制御周期)
# 配信元: ypspur_ros2 / robot_navigator (robot_simulator_node)
# 備考: robot_navigator は 0.5 秒の watchdog で監視し、途絶すると停止指令を出す。
std_msgs/Header header
float64 max_linear_velocity
float64 max_angular_velocity
float64 linear_acceleration
float64 linear_deceleration
float64 angular_acceleration
