# Rocket payload
The rocket payload have 3 main functionality, send the position from gps data, send the atitude (roll, pitch, yaw) extracted from 9-DOF IMU, send picture from VC0706 camera. The gps is parse from ublox protocol. It contains latitude, longitude, height, and wind speed. The 9-DOF IMU is stranslated to roll-pitch-yaw using combination of Madgwick and Mahony filter. The VC0706 is a jpeg cammera censor. The data will be sent to ground station if the ground station create a call. It also save the picture inside memory card for second send attempt. All data is transmitted using xbee 900MHz wireless communication module.

All program is compiled on top of mbed-os and tested on STM32 F440. Please extract mbed-os before recompiling. The zip of tested mbed os can be downloaded [here](https://drive.google.com/file/d/1gtyZMu3_KcnWADjW88rNP_wWM9JfJ9S3/view?usp=sharing)