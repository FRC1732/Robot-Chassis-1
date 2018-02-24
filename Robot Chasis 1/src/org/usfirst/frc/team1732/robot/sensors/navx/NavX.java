package org.usfirst.frc.team1732.robot.sensors.navx;

import org.usfirst.frc.team1732.robot.sensors.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX extends GyroBase {

	private final AHRS navx;

	public NavX(boolean zeroAtStart, AHRS navx) {
		super(false);
		this.navx = navx;
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
	protected void zero() {
		navx.zeroYaw();
	}

	public void sendNavXData() {
		String imu = "IMU/";
		/* Display 6-axis Processed Angle Data */
		SmartDashboard.putBoolean(imu + "IMU_Connected", navx.isConnected());
		SmartDashboard.putBoolean(imu + "IMU_IsCalibrating", navx.isCalibrating());
		SmartDashboard.putNumber(imu + "IMU_Yaw", navx.getYaw());
		SmartDashboard.putNumber(imu + "IMU_Pitch", navx.getPitch());
		SmartDashboard.putNumber(imu + "IMU_Roll", navx.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires */
		/* magnetometer calibration to be useful) */

		// SmartDashboard.putNumber(imu + "IMU_CompassHeading",
		// navx.getCompassHeading());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful) */
		// SmartDashboard.putNumber(imu + "IMU_FusedHeading", navx.getFusedHeading());

		/* These functions are compatible w/the WPI Gyro Class, providing a simple */
		/* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

		SmartDashboard.putNumber(imu + "IMU_TotalYaw", navx.getAngle());
		SmartDashboard.putNumber(imu + "IMU_Angle", Sensors.convertTotalAngle(navx.getAngle()));
		SmartDashboard.putNumber(imu + "IMU_YawRateDPS", navx.getRate());

		/* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

		SmartDashboard.putNumber(imu + "IMU_Accel_X", navx.getWorldLinearAccelX());
		SmartDashboard.putNumber(imu + "IMU_Accel_Y", navx.getWorldLinearAccelY());
		SmartDashboard.putNumber(imu + "IMU_Accel_Z", navx.getWorldLinearAccelZ());
		SmartDashboard.putBoolean(imu + "IMU_IsMoving", navx.isMoving());
		SmartDashboard.putBoolean(imu + "IMU_IsRotating", navx.isRotating());

		/* Display estimates of velocity/displacement. Note that these values are */
		/* not expected to be accurate enough for estimating robot position on a */
		/* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
		/* of these errors due to single (velocity) integration and especially */
		/* double (displacement) integration. */

		// SmartDashboard.putNumber(imu + "Velocity_X", navx.getVelocityX());
		// SmartDashboard.putNumber(imu + "Velocity_Y", navx.getVelocityY());
		// SmartDashboard.putNumber(imu + "Velocity_Z", navx.getVelocityZ());
		// SmartDashboard.putNumber(imu + "Displacement_X", navx.getDisplacementX());
		// SmartDashboard.putNumber(imu + "Displacement_Y", navx.getDisplacementY());
		// SmartDashboard.putNumber(imu + "Displacement_Z", navx.getDisplacementZ());

		/* Display Raw Gyro/Accelerometer/Magnetometer Values */
		/* NOTE: These values are not normally necessary, but are made available */
		/* for advanced users. Before using this data, please consider whether */
		/* the processed data (see above) will suit your needs. */

		SmartDashboard.putNumber(imu + "RawGyro_X", navx.getRawGyroX());
		SmartDashboard.putNumber(imu + "RawGyro_Y", navx.getRawGyroY());
		SmartDashboard.putNumber(imu + "RawGyro_Z", navx.getRawGyroZ());
		// SmartDashboard.putNumber(imu + "RawAccel_X", navx.getRawAccelX());
		// SmartDashboard.putNumber(imu + "RawAccel_Y", navx.getRawAccelY());
		// SmartDashboard.putNumber(imu + "RawAccel_Z", navx.getRawAccelZ());
		// SmartDashboard.putNumber(imu + "RawMag_X", navx.getRawMagX());
		// SmartDashboard.putNumber(imu + "RawMag_Y", navx.getRawMagY());
		// SmartDashboard.putNumber(imu + "RawMag_Z", navx.getRawMagZ());
		SmartDashboard.putNumber(imu + "IMU_Temp_C", navx.getTempC());

		/* Omnimount Yaw Axis Information */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
		AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
		SmartDashboard.putString(imu + "YawAxisDirection", yaw_axis.up ? "Up" : "Down");
		SmartDashboard.putNumber(imu + "YawAxis", yaw_axis.board_axis.getValue());

		/* Sensor Board Information */
		SmartDashboard.putString(imu + "FirmwareVersion", navx.getFirmwareVersion());

		/* Quaternion Data */
		/* Quaternions are fascinating, and are the most compact representation of */
		/* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
		/* from the Quaternions. If interested in motion processing, knowledge of */
		/* Quaternions is highly recommended. */
		// SmartDashboard.putNumber(imu + "QuaternionW", navx.getQuaternionW());
		// SmartDashboard.putNumber(imu + "QuaternionX", navx.getQuaternionX());
		// SmartDashboard.putNumber(imu + "QuaternionY", navx.getQuaternionY());
		// SmartDashboard.putNumber(imu + "QuaternionZ", navx.getQuaternionZ());

		/* Connectivity Debugging Support */
		SmartDashboard.putNumber(imu + "IMU_Byte_Count", navx.getByteCount());
		SmartDashboard.putNumber(imu + "IMU_Update_Count", navx.getUpdateCount());
	}

}
