package org.usfirst.frc.team1732.robot.drivercontrol;

public class RobotDriveBase {
	public static final double kDefaultDeadband = 0.02;
	public static final double kDefaultMaxOutput = 1.0;

	protected double m_deadband = kDefaultDeadband;
	protected double m_maxOutput = kDefaultMaxOutput;

	/**
	 * Change the default value for deadband scaling. The default value is
	 * {@value #kDefaultDeadband}. Values smaller then the deadband are set to 0,
	 * while values larger then the deadband are scaled from 0.0 to 1.0. See
	 * {@link #applyDeadband}.
	 *
	 * @param deadband
	 *            The deadband to set.
	 */
	public void setDeadband(double deadband) {
		m_deadband = deadband;
	}

	/**
	 * Configure the scaling factor for using drive methods with motor controllers
	 * in a mode other than PercentVbus or to limit the maximum output.
	 *
	 * <p>
	 * The default value is {@value #kDefaultMaxOutput}.
	 *
	 * @param maxOutput
	 *            Multiplied with the output percentage computed by the drive
	 *            functions.
	 */
	public void setMaxOutput(double maxOutput) {
		m_maxOutput = maxOutput;
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
	 * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
	 *
	 * @param value
	 *            value to clip
	 * @param deadband
	 *            range around zero
	 */
	protected double applyDeadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

}