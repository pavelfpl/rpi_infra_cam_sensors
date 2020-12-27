# rpi_infra_cam_sensors
Various I2C user space drivers for Raspberry Pi (Zero) including <b>Melexis Infra Arrays.</b>  
C++ library was written as test platform for Strato Balloon Flights.  
No visual output - only saving to a file for subsequent evaluation!  

## Included drivers:
- Humidity Sensor BME280
- MPU-6000 3-Axis Gyroscope/Accelerometer driver 
- Panasonic GridEye AMG8833 Infrared Array Sensor 8x8 driver
- Melexis MLX90621 Low Noise High Speed Far Infrared Array Camera (16x4)
- Melexis MLX90640 Far Infrared Thermal Sensor (32x24)

## Dependencies

Boost, TinyXML  
<b>mlx90640-library</b> is used for MLX90640. Compiled shared library libMLX90640_API.so is included.  
See: https://github.com/melexis/mlx90640-library

## Building & running

I2C device (e.g. /dev/i2c-0 or /dev/i2c-1) can be selected in <b>i2c_drive.c</b>

make
Run ./i2c_driver
Run as systemd service : i2c.service

## Sensor save format

BME280 example format:

[Cycle: 1] timestamp: 2020-02-07T22:36:40   
Temperature [C]: 13.9598  
Temperature [F]: 57.1276  
Pressure [hPa]: 1009.78  
Relative humidity  [RH]: 49.8733  

Melexis MLX90621 example format: 

[Cycle: 1] timestamp: 2020-02-07T22:36:41
MLX90621 resolution: 3
MLX90621 PtaT: 24677
MLX90621 k_t1_scale: 8
MLX90621 k_t2_scale: 21
MLX90621 CpiX: -145
MLX90621 data [deg C]: -18.712822,4.653984,11.790787,12.570309,-23.141027,1.917633,9.477691,9.574051,-25.999409,4.333687,7.723080,8.903610,-30.026731,-15.960690,4.759425,3.249201,-31.725107,-30.930277,-4.064466,-18.205719,-31.944046,-32.308559,-31.901194,-31.742537,-32.465351,-32.454033,-32.549442,-32.069061,-32.870094,-32.998779,-32.645477,-32.624294,-32.298100,-32.770611,-32.976776,-32.384335,-32.883499,-32.375179,-32.566467,-31.981077,-32.396591,-32.086876,-32.294159,-32.129192,-31.649887,-31.779812,-31.699854,-31.604929,-31.303516,-31.146633,-31.537132,-31.477524,-30.234663,-30.321075,-30.737177,-30.452631,-29.148190,-28.749817,-29.052919,-29.386505,-26.139986,-26.791670,-27.380850,-26.880142
MLX90621 RAW data: -361,-203,-124,-123,-427,-230,-155,-143,-475,-213,-172,-137,-561,-436,-201,-217,-586,-626,-315,-436,-633,-660,-654,-587,-656,-689,-674,-616,-663,-706,-698,-614,-663,-696,-690,-616,-647,-691,-669,-597,-614,-658,-650,-570,-581,-621,-612,-553,-530,-569,-559,-504,-479,-511,-503,-461,-425,-451,-440,-396,-357,-396,-383,-353
