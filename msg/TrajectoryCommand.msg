# TrajectoryCommand.msg

# start point and end point (mm)
float64[3] start_position
float64[3] end_position
# Eular angles (rad)
float64[3] start_euler_xyz
float64[3] end_euler_xyz
# v (mm/s)
float64[3] v_start_position
float64[3] v_end_position
# w (rad/s)
float64[3] w_start_euler
float64[3] w_end_euler
# duration (s)
float64 duration
# frequency (单位: Hz)
float64 frequency

