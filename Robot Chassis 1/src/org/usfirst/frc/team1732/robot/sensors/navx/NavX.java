package org.usfirst.frc.team1732.robot.sensors.navx;

import org.usfirst.frc.team1732.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

public class NavX extends GyroBase {

	private final AHRS navx;

	public NavX(boolean zeroAtStart, AHRS navx) {
		super(false);
		this.navx = navx;
		System.out.println("navx update rate: " + navx.getActualUpdateRate());
		// if (zeroAtStart) zero();
	}

	@Override
	public double getAngle() {
		return navx.getYaw();
	}

	@Override
	public double getTotalAngle() {
		return navx.getAngle();
	}

	@Override
	public void zero() {
		navx.zeroYaw();
	}

	public void addNavXData() {
		String imu = "IMU/";
		/* Display 6-axis Processed Angle Data */
		Robot.dash.add(imu + "IMU_Connected", navx::isConnected);
		Robot.dash.add(imu + "IMU_IsCalibrating", navx::isCalibrating);
		Robot.dash.add(imu + "IMU_Yaw", navx::getYaw);
		Robot.dash.add(imu + "IMU_Pitch", navx::getPitch);
		Robot.dash.add(imu + "IMU_Roll", navx::getRoll);

		/* Display tilt-corrected, Magnetometer-based heading (requires */
		/* magnetometer calibration to be useful) */

		// SmartDashboard.putNumber(imu + "IMU_CompassHeading",
		// navx.getCompassHeading);

		/* Display 9-axis Heading (requires magnetometer calibration to be useful) */
		// SmartDashboard.putNumber(imu + "IMU_FusedHeading", navx.getFusedHeading);

		/* These functions are compatible w/the WPI Gyro Class, providing a simple */
		/* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

		Robot.dash.add(imu + "IMU_TotalYaw", navx::getAngle);
		// Robot.dash.add(imu + "IMU_Angle", Sensors::convertTotalAngle(navx.getAngle));
		Robot.dash.add(imu + "IMU_YawRateDPS", navx::getRate);

		/* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

		Robot.dash.add(imu + "IMU_Accel_X", navx::getWorldLinearAccelX);
		Robot.dash.add(imu + "IMU_Accel_Y", navx::getWorldLinearAccelY);
		Robot.dash.add(imu + "IMU_Accel_Z", navx::getWorldLinearAccelZ);
		Robot.dash.add(imu + "IMU_IsMoving", navx::isMoving);
		Robot.dash.add(imu + "IMU_IsRotating", navx::isRotating);

		/* Display estimates of velocity/displacement. Note that these values are */
		/* not expected to be accurate enough for estimating robot position on a */
		/* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
		/* of these errors due to single (velocity) integration and especially */
		/* double (displacement) integration. */

		// Robot.dash.add(imu + "Velocity_X", navx.getVelocityX);
		// Robot.dash.add(imu + "Velocity_Y", navx.getVelocityY);
		// Robot.dash.add(imu + "Velocity_Z", navx.getVelocityZ);
		// Robot.dash.add(imu + "Displacement_X", navx.getDisplacementX);
		// Robot.dash.add(imu + "Displacement_Y", navx.getDisplacementY);
		// Robot.dash.add(imu + "Displacement_Z", navx.getDisplacementZ);

		/* Display Raw Gyro/Accelerometer/Magnetometer Values */
		/* NOTE: These values are not normally necessary, but are made available */
		/* for advanced users. Before using this data, please consider whether */
		/* the processed data (see above) will suit your needs. */

		Robot.dash.add(imu + "RawGyro_X", navx::getRawGyroX);
		Robot.dash.add(imu + "RawGyro_Y", navx::getRawGyroY);
		Robot.dash.add(imu + "RawGyro_Z", navx::getRawGyroZ);
		// Robot.dash.add(imu + "RawAccel_X", navx.getRawAccelX);
		// Robot.dash.add(imu + "RawAccel_Y", navx.getRawAccelY);
		// Robot.dash.add(imu + "RawAccel_Z", navx.getRawAccelZ);
		// Robot.dash.add(imu + "RawMag_X", navx.getRawMagX);
		// Robot.dash.add(imu + "RawMag_Y", navx.getRawMagY);
		// Robot.dash.add(imu + "RawMag_Z", navx.getRawMagZ);
		Robot.dash.add(imu + "IMU_Temp_C", navx::getTempC);

		/* Omnimount Yaw Axis Information */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
		AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
		// SmartDashboard.putString(imu + "YawAxisDirection", yaw_axis.up ? "Up" :
		// "Down");
		Robot.dash.add(imu + "YawAxis", yaw_axis.board_axis::getValue);

		/* Sensor Board Information */
		Robot.dash.add(imu + "FirmwareVersion", navx::getFirmwareVersion);

		/* Quaternion Data */
		/* Quaternions are fascinating, and are the most compact representation of */
		/* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
		/* from the Quaternions. If interested in motion processing, knowledge of */
		/* Quaternions is highly recommended. */
		// Robot.dash.add(imu + "QuaternionW", navx.getQuaternionW);
		// Robot.dash.add(imu + "QuaternionX", navx.getQuaternionX);
		// Robot.dash.add(imu + "QuaternionY", navx.getQuaternionY);
		// Robot.dash.add(imu + "QuaternionZ", navx.getQuaternionZ);

		/* Connectivity Debugging Support */
		Robot.dash.add(imu + "IMU_Byte_Count", navx::getByteCount);
		Robot.dash.add(imu + "IMU_Update_Count", navx::getUpdateCount);
	}

}
