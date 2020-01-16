# message 325
# Contains the RAWIMU data corrected for gravity, 
# the earth's rotation and estimated sensor errors.
novatel_msgs/CommonHeader header

int32 week
float64 gpssec

uint32 imu_status

# Gyro values are instantaneous and in radians, must
# multiply by datarate to get rad/s.
int32 z_accel
int32 my_accel
int32 x_accel

int32 z_gyro
int32 my_gyro
int32 x_gyro
