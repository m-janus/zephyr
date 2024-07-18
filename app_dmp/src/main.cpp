/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "../dmp/src/ICM_20948.h"
#include <math.h>
#include <stdio.h>

#define PI 3.141592654
#define I2C_DEV_NODE DT_NODELABEL(i2c1)
#define I2C_ADDR 0x69

ICM_20948_Device_t icm_dev;


ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    int ret = i2c_write(i2c_dev, buf, sizeof(buf), I2C_ADDR);
    if (ret != 0) {
        return ICM_20948_Stat_Err; // Adjust error handling as necessary
    }

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);

    // Write the register address
    int ret = i2c_write(i2c_dev, &reg, 1, I2C_ADDR);
    if (ret != 0) {
        return ICM_20948_Stat_Err; // Adjust error handling as necessary
    }

    // Read the data from the register
    ret = i2c_read(i2c_dev, buff, len, I2C_ADDR);
    if (ret != 0) {
        return ICM_20948_Stat_Err; // Adjust error handling as necessary
    }

    return ICM_20948_Stat_Ok;
}

const ICM_20948_Serif_t mySerif = {
    my_write_i2c, // write
    my_read_i2c,  // read
    NULL,
};

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object


// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	// Initialize myICM
	// ICM_20948_init_struct(&myICM);
	// printf("ICM struct init \n");

	// // Link the serif
	// ICM_20948_link_serif(&myICM, &mySerif);
	// printf("ICM link serif \n");

    // Get the I2C device binding
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C: Device driver not found.\n");
        return -ENODEV;
    }

	/////////////////////////////////////////////////////////// demo DMP start

	/////////////////////////////// setup

	  bool initialized = false;
	while (!initialized)
	{

		// Initialize the ICM-20948
		// If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
	#ifdef USE_SPI
		myICM.begin(CS_PIN, SPI_PORT);
	#else
		myICM.begin(i2c_dev, AD0_VAL);
	#endif

    printf("Initialization of the sensor returned: ");
    printf(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      printf("Trying again...\n");
      k_sleep(K_MSEC(500));
    }
    else
    {
      initialized = true;
    }
  }

	printf("Device connected!\n");

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor (Quat6)
//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable additional sensors / features
//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok); // DMP_header_bitmap_Accel
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok); // DMP_header_bitmap_Gyro
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok); //  DMP_header_bitmap_Compass, but doesnt change value
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok); //  

//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz DMP_header_bitmap_Accel
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz DMP_header_bitmap_Gyro
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz  DMP_header_bitmap_Compass, but doesnt change value

    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz  DMP_header_bitmap_Compass, but doesnt change value

//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Geomag, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    printf("DMP enabled!\n");
  }
  else
  {
    printf("Enable DMP failed!\n");
    printf("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\n");
    while (1)
      ; // Do nothing more
  }

	////////////////////////////////// setup end

	/// looooop start 

	while(1)
	{
		  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for orientation data (Quat9)
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
    //   double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    //   double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    //   double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

	// 	printf("quat Q1: %d Q2: %d Q3: %d\n", q1, q2, q3);

	      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

	    char bufRoll[32];
		char bufPitch[32];
	    char bufYaw[32];

    	snprintf(bufRoll, sizeof(bufRoll), "%f", roll);
    	snprintf(bufPitch, sizeof(bufPitch), "%f", pitch);
    	snprintf(bufYaw, sizeof(bufYaw), "%f", yaw);

		printf("Roll: %s Pitch: %s Yaw: %s\n", bufRoll, bufPitch, bufYaw);

    }


    if ((data.header & DMP_header_bitmap_Quat9) > 0) // Check for orientation data (Quat9)
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
    //   double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    //   double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    //   double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

	// 	printf("quat Q1: %d Q2: %d Q3: %d\n", q1, q2, q3);

	      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

	    char bufRoll[32];
		  char bufPitch[32];
	    char bufYaw[32];

    	snprintf(bufRoll, sizeof(bufRoll), "%f", roll);
    	snprintf(bufPitch, sizeof(bufPitch), "%f", pitch);
    	snprintf(bufYaw, sizeof(bufYaw), "%f", yaw);

		printf("Roll: %s Pitch: %s Yaw: %s\n", bufRoll, bufPitch, bufYaw);

    }


    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      float acc_x = (float)data.Raw_Accel.Data.X / 8192.0; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y / 8192.0;
      float acc_z = (float)data.Raw_Accel.Data.Z / 8192.0;


	    char bufX[32];
		  char bufY[32];
	    char bufZ[32];
    	snprintf(bufX, sizeof(bufX), "%f", acc_x);
    	snprintf(bufY, sizeof(bufY), "%f", acc_y);
    	snprintf(bufZ, sizeof(bufZ), "%f", acc_z);

		printf("Accel X: %s Y: %s Z: %s\n", bufX, bufY, bufZ);
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float y = (float)data.Raw_Gyro.Data.Y;
      float z = (float)data.Raw_Gyro.Data.Z;

		printf("Gyro X: %d Y: %d Z: %d\n", x, y, z);
    }

    if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
    {
      float x = (float)data.Geomag.Data.Q1; // Extract the compass data
      float y = (float)data.Geomag.Data.Q2;
      float z = (float)data.Geomag.Data.Q3;

		printf("Compass X: %d Y: %d Z: %d\n", x, y, z);

    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    k_sleep(K_MSEC(10));
  }
	}

	////////////////////////////////////////////////////////// demo DMP end

	return 0;
}
