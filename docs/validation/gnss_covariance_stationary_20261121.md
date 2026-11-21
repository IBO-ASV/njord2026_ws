# UM982 stationary covariance measurement (2026-11-21)

A live, stationary 300 s observation on AGX domain 126 collected 6001
NavSatFix and 6000 heading messages. The horizontal coordinates were converted
to local ENU relative to the first fix. No raw positions were saved.

| Quantity | Standard deviation | Variance |
| --- | ---: | ---: |
| east | 0.205936 m | 0.042409636 m² |
| north | 0.170792 m | 0.029169907 m² |
| height | 0.121758 m | 0.014825011 m² |
| yaw | 15.446865° | 0.072683434 rad² |

Yaw uses circular differences. The current THS heading has yaw only: the zero
pitch in its quaternion is not a measurement. The onboard map EKF is in 2D
mode and fuses GNSS XY and yaw only, not Z or pitch. Configured position
variances are minimums; when GST reports higher variance it is retained.
Likewise the configured yaw variance is a minimum. Off-diagonal terms on the
calibrated position input are zero. This single static observation includes
slow drift and is not an estimate of sensor bias, accuracy, or behavior under
motion; yaw and antenna/RTK conditions require further investigation.

A separate 75 s concurrent shadow EKF compared its map yaw with a 3° input
standard deviation: raw yaw 0.700°, original EKF 0.694°, 3° EKF 0.638°, and
GLIM local yaw 0.020° standard deviation. No physical motion was requested.
The shadow EKF did not publish TF or command actuators. The 3° setting alone
did not yield GLIM-level stability. The GNSS Map Telemetry heading display
uses raw GNSS heading, so the map EKF covariance does not smooth that icon.
