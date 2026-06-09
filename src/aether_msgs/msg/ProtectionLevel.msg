# The breathing integrity bound (GNSS-RAIM analogue applied to the vision aid,
# on the RELATIVE / segment-RPE quantity DP7 grades).
std_msgs/Header header
float64 pl_operational      # k_op = 2.45 bound (95% / operational)
float64 pl_dal_c            # k_ffd = 6.74 bound (DAL-C / DO-178C-as-if)
float64 horizontal_pl       # 2D horizontal PL (the ellipse radius)
float64 covariance_trace    # trace of the 3x3 position covariance (m^2)
float64[3] semi_axes        # operational PL semi-axes (m) [major, minor, vertical], for RViz
float64 ellipse_yaw         # yaw (rad) of the horizontal error-ellipse major axis, odom frame
