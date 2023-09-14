# IMU_pose

QMC5883 magnetometer along with the 6-DOF IMU BMI160 was used to achieve accurate measurements of roll, pitch, north heading values with tilt compensation. Note that the QMC5883 needs hard/soft iron calibration, along with location declination, location declination changes every year so the declination has to be updated over time for accurate results. 

The QMC5883 has to be at the same direction as the BMI160 is otherwise you need to make changes to the roll/pitch angles before they are used to compensate the tilt for the north heading: 

![WhatsApp Image 2023-09-10 at 11 33 59 PM](https://github.com/Hasan-Amkieh/IMU_pose/assets/46199105/951e402e-fff5-4903-8133-43b234c0452a)

Live Demonstration using Processing IDE:

![GIFMaker_me](https://github.com/Hasan-Amkieh/IMU_pose/assets/46199105/6625196a-d1bb-44ef-a631-55634ebf0c0a)