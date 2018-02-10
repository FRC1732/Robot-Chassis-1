package org.usfirst.frc.team1732.robot.drivercontrol;

import org.usfirst.frc.team1732.robot.math.Util;

public abstract class RobotDriveBase {
	public static final double kDefaultDeadband = 0.0;
	public static final double kDefaultMinOutput = 0.0;
	public static final double kDefaultMaxOutput = 1.0;

	protected final double m_minOut;
	protected final double m_deadband;
	protected final double m_maxOut;

	public RobotDriveBase(double minOut, double maxOut, double deadband) {
		m_minOut = minOut;
		m_deadband = deadband;
		m_maxOut = maxOut;
	}

	public RobotDriveBase() {
		this(kDefaultMinOutput, kDefaultMaxOutput, kDefaultDeadband);
	}

	/**
	 * Limit motor values to the -1.0 to +1.0 range.
	 */
	protected double limit(double value) {
		if (value > 1.0) {
			return 1.0;
		}
		if (value < -1.0) {
			return -1.0;
		}
		return value;
	}

	/**
	 * Returns 0.0 if the given value is within the specified range around zero. The
	 * remaining range between the deadband and 1.0 is scaled from 0 to 1.0.
	 *
	 * @param value
	 *            value to clip
	 * @param deadband
	 *            range around zero
	 */
	protected double applyDeadband(double value) {
		if (Math.abs(value) > m_deadband) {
			if (value > 0.0) {
				return (value - m_deadband) / (1.0 - m_deadband);
			} else {
				return (value + m_deadband) / (1.0 - m_deadband);
			}
		} else {
			return 0.0;
		}
	}

	protected double applyMinMax(double value) {
		return Math.copySign(Util.interpolate(m_minOut, m_maxOut, Math.abs(value)), value);
	}

}