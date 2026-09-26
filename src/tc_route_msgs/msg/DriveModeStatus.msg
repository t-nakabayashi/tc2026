# 自律・手動の切替状態と、最終的に出力された速度指令の由来を通知する。
#
# 配信様式: stream (20 Hz, drive_cmd_mux_node の publish_rate_hz)
# 配信元: drive_mode_manager (drive_cmd_mux_node)

std_msgs/Header header

uint8 MODE_AUTONOMOUS=1
uint8 MODE_MANUAL=2
uint8 mode

uint8 SOURCE_ZERO=0
uint8 SOURCE_AUTONOMOUS_CMD=1
uint8 SOURCE_MANUAL_CMD=2
uint8 output_source

bool joy_available
bool l1_pressed
bool ps_button_pressed
float32 ps_hold_progress_s

bool manual_input_active
bool manual_cmd_alive
bool autonomous_cmd_alive

bool auto_resume_pending
float32 auto_resume_remaining_s

float32 pending_autonomous_linear_x
float32 pending_autonomous_angular_z

float32 output_linear_x
float32 output_angular_z

string reason
