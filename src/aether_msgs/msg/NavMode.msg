# A.E.T.H.E.R degradation state
uint8 NOMINAL    = 0     # vision + IMU, bound tight
uint8 DEGRADED   = 1     # vision weak (low features / growing covariance), bound widening
uint8 INERTIAL   = 2     # vision dead, INS-only dead-reckon, bound blooming
uint8 RE_ACQUIRE = 3     # vision returning, bound contracting
uint8 mode
