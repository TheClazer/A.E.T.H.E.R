# OPTIONAL / STRETCH: per-feature MSCKF internals (forked OpenVINS UpdaterMSCKF::update).
# The live demo does NOT depend on this; it is computed offline if time permits.
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose   # 6x6 covariance = state uncertainty
float64[3] velocity                     # body-frame linear velocity (INS spine)
uint32 num_slam_features
uint32 num_msckf_features
float64 cond_number                     # condition number of the update info matrix
float64 disparity_px                    # median feature disparity (vision-aiding strength)
