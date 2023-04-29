# This is the LMAR Project for Capstone 2022-2023 at UML

This package contains some simple drivers for the different components connected to the Raspberry Pi 3B+ used to control LMAR.

The key components is the startup launch file, "lmar_startup.launch" which starts up 4 different drivers. These drivers are:

    1. GPIO Motor Controller
    2. GPIO Leak Sensor Detector
    3. NMEA_NavSat_Driver (http://wiki.ros.org/nmea_navsat_driver)
    4. BNO055 IMU I2C Driver (https://github.com/dheera/ros-imu-bno055)

1. The GPIO Motor Controller subscribes to the topic 'cmd_vel', which is published by the 'teleop_twist_keyboard' package from a remote computer. From this, we expect the user to only attempt to drive forwards or backwards, or turn left/right, no mixed translation and rotation. 

2. The GPIO Leak Sensor Detector simply runs a control loop, polling each GPIO pin for a high signal, which will be caused due to the sensors touching water. In this case, it will log an error in ros, naming which cylinder has the leak.

3. The NMEA_NavSat_Driver is a public rospackage, which published to three topics, 'fix', which is the position from the GPS, 'vel', which is the velocity from the gps, and 'time_reference', which is the timestamp from the GPS.

4. The BNO055 IMU I2C Driver can be found at the above link. This package must be installed alongside the lmar_ros package. This provides numerous types of IMU data. Each topic can be found on the github linked above, and the data contained inside these topics.