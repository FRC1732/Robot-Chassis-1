package org.usfirst.frc.team1732.robot.sensors.navx;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavXData {

	public static void sendNavXData(AHRS ahrs) {
		String imu = "IMU/";
		/* Display 6-axis Processed Angle Data */
		SmartDashboard.putBoolean(imu + "IMU_Connected", ahrs.isConnected());
		SmartDashboard.putBoolean(imu + "IMU_IsCalibrating", ahrs.isCalibrating());
		SmartDashboard.putNumber(imu + "IMU_Yaw", ahrs.getYaw());
		SmartDashboard.putNumber(imu + "IMU_Pitch", ahrs.getPitch());
		SmartDashboard.putNumber(imu + "IMU_Roll", ahrs.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires */
		/* magnetometer calibration to be useful) */

		// SmartDashboard.putNumber(imu + "IMU_CompassHeading",
		// ahrs.getCompassHeading());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful) */
		// SmartDashboard.putNumber(imu + "IMU_FusedHeading", ahrs.getFusedHeading());

		/* These functions are compatible w/the WPI Gyro Class, providing a simple */
		/* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

		SmartDashboard.putNumber(imu + "IMU_TotalYaw", ahrs.getAngle());
		SmartDashboard.putNumber(imu + "IMU_YawRateDPS", ahrs.getRate());

		/* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

		SmartDashboard.putNumber(imu + "IMU_Accel_X", ahrs.getWorldLinearAccelX());
		SmartDashboard.putNumber(imu + "IMU_Accel_Y", ahrs.getWorldLinearAccelY());
		SmartDashboard.putNumber(imu + "IMU_Accel_Z", ahrs.getWorldLinearAccelZ());
		SmartDashboard.putBoolean(imu + "IMU_IsMoving", ahrs.isMoving());
		SmartDashboard.putBoolean(imu + "IMU_IsRotating", ahrs.isRotating());

		/* Display estimates of velocity/displacement. Note that these values are */
		/* not expected to be accurate enough for estimating robot position on a */
		/* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
		/* of these errors due to single (velocity) integration and especially */
		/* double (displacement) integration. */

		// SmartDashboard.putNumber(imu + "Velocity_X", ahrs.getVelocityX());
		// SmartDashboard.putNumber(imu + "Velocity_Y", ahrs.getVelocityY());
		// SmartDashboard.putNumber(imu + "Velocity_Z", ahrs.getVelocityZ());
		// SmartDashboard.putNumber(imu + "Displacement_X", ahrs.getDisplacementX());
		// SmartDashboard.putNumber(imu + "Displacement_Y", ahrs.getDisplacementY());
		// SmartDashboard.putNumber(imu + "Displacement_Z", ahrs.getDisplacementZ());

		/* Display Raw Gyro/Accelerometer/Magnetometer Values */
		/* NOTE: These values are not normally necessary, but are made available */
		/* for advanced users. Before using this data, please consider whether */
		/* the processed data (see above) will suit your needs. */

		// SmartDashboard.putNumber(imu + "RawGyro_X", ahrs.getRawGyroX());
		// SmartDashboard.putNumber(imu + "RawGyro_Y", ahrs.getRawGyroY());
		// SmartDashboard.putNumber(imu + "RawGyro_Z", ahrs.getRawGyroZ());
		// SmartDashboard.putNumber(imu + "RawAccel_X", ahrs.getRawAccelX());
		// SmartDashboard.putNumber(imu + "RawAccel_Y", ahrs.getRawAccelY());
		// SmartDashboard.putNumber(imu + "RawAccel_Z", ahrs.getRawAccelZ());
		// SmartDashboard.putNumber(imu + "RawMag_X", ahrs.getRawMagX());
		// SmartDashboard.putNumber(imu + "RawMag_Y", ahrs.getRawMagY());
		// SmartDashboard.putNumber(imu + "RawMag_Z", ahrs.getRawMagZ());
		SmartDashboard.putNumber(imu + "IMU_Temp_C", ahrs.getTempC());

		/* Omnimount Yaw Axis Information */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
		AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
		SmartDashboard.putString(imu + "YawAxisDirection", yaw_axis.up ? "Up" : "Down");
		SmartDashboard.putNumber(imu + "YawAxis", yaw_axis.board_axis.getValue());

		/* Sensor Board Information */
		SmartDashboard.putString(imu + "FirmwareVersion", ahrs.getFirmwareVersion());

		/* Quaternion Data */
		/* Quaternions are fascinating, and are the most compact representation of */
		/* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
		/* from the Quaternions. If interested in motion processing, knowledge of */
		/* Quaternions is highly recommended. */
		// SmartDashboard.putNumber(imu + "QuaternionW", ahrs.getQuaternionW());
		// SmartDashboard.putNumber(imu + "QuaternionX", ahrs.getQuaternionX());
		// SmartDashboard.putNumber(imu + "QuaternionY", ahrs.getQuaternionY());
		// SmartDashboard.putNumber(imu + "QuaternionZ", ahrs.getQuaternionZ());

		/* Connectivity Debugging Support */
		SmartDashboard.putNumber(imu + "IMU_Byte_Count", ahrs.getByteCount());
		SmartDashboard.putNumber(imu + "IMU_Update_Count", ahrs.getUpdateCount());
	}
}