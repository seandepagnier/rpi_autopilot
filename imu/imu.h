enum {ACCELX, ACCELY, ACCELZ, GYROX, GYROY, GYROZ, MAGX, MAGY, MAGZ, BARO, SENSOR_COUNT};

/* called once at start to connect and configure,
   0 on success -1 on failure */
int imu_init(const char *config_file);

void imu_read(float s[9]); /* calibrated sensor reading */

void imu_yaw_offset(float angle); /* correct for yaw offset */
void imu_level(); /* call when level to align */

void imu_quaternion(float q[4]);
void imu_orientation(float a[3]); /* pitch roll yaw */
void imu_rate(float a[3]); /* pitch roll yaw rates */
