package org.usfirst.frc.team1732.robot.drivercontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;

/**
 * A class for driving differential drive/skid-steer drive platforms such as the
 * Kit of Parts drive base, "tank drive", or West Coast Drive.
 *
 * <p>
 * These drive bases typically have drop-center / skid-steer with two or more
 * wheels per side (e.g., 6WD or 8WD). This class takes a TalonSRX per side. For
 * four and six motor drivetrains, construct and pass in
 * {@link edu.wpi.first.wpilibj.SpeedControllerGroup} instances as follows.
 * 
 */

public class DifferentialDrive extends RobotDriveBase {
	public static final double kDefaultQuickStopThreshold = 0.2;
	public static final double kDefaultQuickStopAlpha = 0.1;

	private TalonSRX m_leftMotor;
	private TalonSRX m_rightMotor;
	private ControlMode controlMode = ControlMode.PercentOutput;

	private double m_quickStopThreshold = kDefaultQuickStopThreshold;
	private double m_quickStopAlpha = kDefaultQuickStopAlpha;
	private double m_quickStopAccumulator = 0.0;
	private boolean m_reported = false;

	/**
	 * Construct a DifferentialDrive.
	 *
	 * <p>
	 * To pass multiple motors per side, use a {@link SpeedControllerGroup}. If a
	 * motor needs to be inverted, do so before passing it in.
	 */
	public DifferentialDrive(TalonSRX leftMotor, TalonSRX rightMotor) {
		m_leftMotor = leftMotor;
		m_rightMotor = rightMotor;
	}

	public void setControlMode(ControlMode mode) {
		controlMode = mode;
	}

	/**
	 * Arcade drive method for differential drive platform. The calculated values
	 * will be squared to decrease sensitivity at low speeds.
	 *
	 * @param xSpeed
	 *            The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *            positive.
	 * @param zRotation
	 *            The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise
	 *            is positive.
	 */
	public void arcadeDrive(double xSpeed, double zRotation) {
		arcadeDrive(xSpeed, zRotation, true);
	}

	/**
	 * Arcade drive method for differential drive platform.
	 *
	 * @param xSpeed
	 *            The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *            positive.
	 * @param zRotation
	 *            The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise
	 *            is positive.
	 * @param squaredInputs
	 *            If set, decreases the input sensitivity at low speeds.
	 */
	public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
		if (!m_reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, 2, tInstances.kRobotDrive_ArcadeStandard);
			m_reported = true;
		}

		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);

		zRotation = limit(zRotation);
		zRotation = applyDeadband(zRotation, m_deadband);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squaredInputs) {
			xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
			zRotation = Math.copySign(zRotation * zRotation, zRotation);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

		if (xSpeed >= 0.0) {
			// First quadrant, else second quadrant
			if (zRotation >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			} else {
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (zRotation >= 0.0) {
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			}
		}

		m_leftMotor.set(controlMode, limit(leftMotorOutput) * m_maxOutput);
		m_rightMotor.set(controlMode, limit(rightMotorOutput) * m_maxOutput);
	}

	/**
	 * Curvature drive method for differential drive platform.
	 *
	 * <p>
	 * The rotation argument controls the curvature of the robot's path rather than
	 * its rate of heading change. This makes the robot more controllable at high
	 * speeds. Also handles the robot's quick turn functionality - "quick turn"
	 * overrides constant-curvature turning for turn-in-place maneuvers.
	 *
	 * @param xSpeed
	 *            The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *            positive.
	 * @param zRotation
	 *            The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise
	 *            is positive.
	 * @param isQuickTurn
	 *            If set, overrides constant-curvature turning for turn-in-place
	 *            maneuvers.
	 */
	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		if (!m_reported) {
			// HAL.report(tResourceType.kResourceType_RobotDrive, 2,
			// tInstances.kRobotDrive_Curvature);
			m_reported = true;
		}

		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);

		zRotation = limit(zRotation);
		zRotation = applyDeadband(zRotation, m_deadband);

		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(xSpeed) < m_quickStopThreshold) {
				m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
						+ m_quickStopAlpha * limit(zRotation) * 2;
			}
			overPower = true;
			angularPower = zRotation;
		} else {
			overPower = false;
			angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1) {
				m_quickStopAccumulator -= 1;
			} else if (m_quickStopAccumulator < -1) {
				m_quickStopAccumulator += 1;
			} else {
				m_quickStopAccumulator = 0.0;
			}
		}

		double leftMotorOutput = xSpeed + angularPower;
		double rightMotorOutput = xSpeed - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		m_leftMotor.set(controlMode, leftMotorOutput * m_maxOutput);
		m_rightMotor.set(controlMode, rightMotorOutput * m_maxOutput);
	}

	/**
	 * Tank drive method for differential drive platform. The calculated values will
	 * be squared to decrease sensitivity at low speeds.
	 *
	 * @param leftSpeed
	 *            The robot's left side speed along the X axis [-1.0..1.0]. Forward
	 *            is positive.
	 * @param rightSpeed
	 *            The robot's right side speed along the X axis [-1.0..1.0]. Forward
	 *            is positive.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		tankDrive(leftSpeed, rightSpeed, true);
	}

	/**
	 * Tank drive method for differential drive platform.
	 *
	 * @param leftSpeed
	 *            The robot left side's speed along the X axis [-1.0..1.0]. Forward
	 *            is positive.
	 * @param rightSpeed
	 *            The robot right side's speed along the X axis [-1.0..1.0]. Forward
	 *            is positive.
	 * @param squaredInputs
	 *            If set, decreases the input sensitivity at low speeds.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
		if (!m_reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, 2, tInstances.kRobotDrive_Tank);
			m_reported = true;
		}

		leftSpeed = limit(leftSpeed);
		leftSpeed = applyDeadband(leftSpeed, m_deadband);

		rightSpeed = limit(rightSpeed);
		rightSpeed = applyDeadband(rightSpeed, m_deadband);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squaredInputs) {
			leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
			rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
		}

		m_leftMotor.set(controlMode, leftSpeed * m_maxOutput);
		m_rightMotor.set(controlMode, rightSpeed * m_maxOutput);
	}

	/**
	 * Sets the QuickStop speed threshold in curvature drive.
	 *
	 * <p>
	 * QuickStop compensates for the robot's moment of inertia when stopping after a
	 * QuickTurn.
	 *
	 * <p>
	 * While QuickTurn is enabled, the QuickStop accumulator takes on the rotation
	 * rate value outputted by the low-pass filter when the robot's speed along the
	 * X axis is below the threshold. When QuickTurn is disabled, the accumulator's
	 * value is applied against the computed angular power request to slow the
	 * robot's rotation.
	 *
	 * @param threshold
	 *            X speed below which quick stop accumulator will receive rotation
	 *            rate values [0..1.0].
	 */
	public void setQuickStopThreshold(double threshold) {
		m_quickStopThreshold = threshold;
	}

	/**
	 * Sets the low-pass filter gain for QuickStop in curvature drive.
	 *
	 * <p>
	 * The low-pass filter filters incoming rotation rate commands to smooth out
	 * high frequency changes.
	 *
	 * @param alpha
	 *            Low-pass filter gain [0.0..2.0]. Smaller values result in slower
	 *            output changes. Values between 1.0 and 2.0 result in output
	 *            oscillation. Values below 0.0 and above 2.0 are unstable.
	 */
	public void setQuickStopAlpha(double alpha) {
		m_quickStopAlpha = alpha;
	}

}