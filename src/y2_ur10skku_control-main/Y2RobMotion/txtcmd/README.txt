# Explanation of "cmd_6D.txt"
-> Each column means x, y, z, wx, wy, wz, desired_lin_vel, desired_ang_vel, holding_time
-> Velocity means the velocity between present and previous points
-> And first row data of velocity will be ignored
-> Unit: mm, degree, mm/s, degree/s, s


# Explanation of "cmd_9D.txt"
-> Each column means x, y, z, wx, wy, wz, fx, fy, fz, desired_lin_vel, desired_ang_vel, holding_time
-> Velocity means the velocity between present and previous points
-> Force means the force between present and previous points
-> And first row data of velocity will be ignored
-> Unit: mm, degree, N, mm/s, degree/s, s


# NOTE!!
-> holding posture must be same with previous posture!!
-> At holding posture, you can maintain the set force
-> Most high priority setting is holding_time 