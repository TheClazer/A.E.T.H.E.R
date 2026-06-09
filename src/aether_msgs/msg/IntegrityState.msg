# integrity_monitor verdict bus
std_msgs/Header header
aether_msgs/ProtectionLevel protection
float64 observability_index   # D in [0,1]: 1 = fully observable, 0 = a direction is blind
float64 nees                  # normalized estimation error squared vs ground truth (sim)
float64 trust                 # continuous 0..1 trust score
bool    bound_valid           # NEES inside chi-square gate -> bound trustworthy
float64 time_since_aiding_s   # seconds since last accepted vision update
aether_msgs/NavMode mode
float64 solution_separation   # horizontal IMU-dead-reckoning vs VIO separation (m), raw
