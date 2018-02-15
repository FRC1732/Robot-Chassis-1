package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.controlutils.ClosedLoopProfile;
import org.usfirst.frc.team1732.robot.controlutils.Feedforward;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.DoubleProfileLoader;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	public final TalonSRX leftTalon1;
	public final TalonSRX rightTalon1;

	public DifferentialDrive drive;

	public final TalonEncoder leftEncoder;
	public final TalonEncoder rightEncoder;

	public static final double INPUT_DEADBAND = 0.025; // 2.5%.
	public static final double MIN_OUTPUT = 0.0;
	public static final double MAX_OUTPUT = 1.0;
	public static final double ENCODER_INCHES_PER_PULSE = 0.002099;

	// Feedforward
	public final Feedforward leftFF = new Feedforward(0.0574657, 0.0089855, 1.6010019, 0.0574657, 0.0089855,
			-1.6010019);
	public final Feedforward rightFF = new Feedforward(0.0560625, 0.0071341, 1.7711541, 0.0560625, 0.0071341,
			-1.7711541);

	// keep in mind for these
	private final ClosedLoopProfile mpGains = new ClosedLoopProfile("MP PID", 0.01 * Math.pow(2, 7), 0.0008, 0.01,
			Feedforward.TALON_SRX_FF_GAIN, 800, 0, 5000, 0);
	public final ClosedLoopProfile leftMPGains = mpGains;
	public final ClosedLoopProfile rightMPGains = mpGains;
	public final ClosedLoopProfile velGains = new ClosedLoopProfile("Closed Loop Profile", 0, 0, 0, 1023 / 7500.0, 0, 0,
			0, 0);

	/*
	 * The following 2 values are determined from the feedforward constants.
	 * 
	 * Max vel is determined by doing (12 - voltIntercept)/2.0 / kV
	 * 
	 * Max acc is determined by doing (12 - voltIntercept)/2.0 / kA
	 * 
	 * This makes it possible to be accelerating at max acceleration near max
	 * velocity (because we cannot exceed a voltage draw of 12V). If we made
	 * acceleration continuous by instead changing 'jerk', then we would be able to
	 * set these slightly higher, depending on how high we set the jerk value. Using
	 * a very high jerk value would not be any different than what we have now, but
	 * using too low of a jerk value would cause the robot to accelerate slowly.
	 */
	public static final double MAX_IN_SEC = 90; // max vel
	public static final double MAX_IN_SEC2 = 500; // max acc

	public static final double ROBOT_WIDTH_IN = 29;
	public static final double EFFECTIVE_ROBOT_WIDTH_IN = ROBOT_WIDTH_IN * 1.0;

	public final DoubleProfileLoader profileManager;

	public Drivetrain() {
		int leftMaster = 1;
		TalonConfiguration config = TalonConfiguration.getDefaultConfig();
		config.enableVoltageCompensation = true;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, config);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false, config), leftTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false, config), leftTalon1);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, config);

		MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true, config), rightTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true, config), rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1, ControlMode.PercentOutput, MIN_OUTPUT, MAX_OUTPUT,
				INPUT_DEADBAND);

		ClosedLoopProfile.applyZeroGainToTalon(leftTalon1, 0, 1);
		ClosedLoopProfile.applyZeroGainToTalon(rightTalon1, 0, 1);
		leftMPGains.applyToTalon(leftTalon1, 0, 0);
		rightMPGains.applyToTalon(rightTalon1, 0, 0);

		// two of these control the shifting. I didn't want to follow the wires
		// new Solenoid(1).set(true);
		// new Solenoid(2).set(true);
		// new Solenoid(3).set(true);

		leftEncoder = new TalonEncoder(leftTalon1, FeedbackDevice.QuadEncoder);
		rightEncoder = new TalonEncoder(rightTalon1, FeedbackDevice.QuadEncoder);
		leftEncoder.setPhase(true);
		rightEncoder.setPhase(true);
		leftEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.zero();
		leftEncoder.zero();

		profileManager = new DoubleProfileLoader(leftTalon1, rightTalon1);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Rate", leftTalon1.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Right Rate", rightTalon1.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Left Velocity", leftEncoder.getRate());
		SmartDashboard.putNumber("Right Velocity", rightEncoder.getRate());
		SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());
		SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
		SmartDashboard.putNumber("Left Voltage", leftTalon1.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Voltage", rightTalon1.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Pulses", leftEncoder.getPulses());
		SmartDashboard.putNumber("Left Pulses", rightEncoder.getPulses());

	}

	public EncoderReader makeRightEncoderReader() {
		return makeRightEncoderReader(false);
	}

	public EncoderReader makeRightEncoderReader(boolean zero) {
		EncoderReader r = rightEncoder.makeReader();
		if (zero)
			r.zero();
		return r;
	}

	public EncoderReader makeLeftEncoderReader() {
		return makeLeftEncoderReader(false);
	}

	public EncoderReader makeLeftEncoderReader(boolean zero) {
		EncoderReader r = leftEncoder.makeReader();
		if (zero)
			r.zero();
		return r;
	}

	public void setStop() {
		drive.tankDrive(0, 0);
	}

	public void setLeft(double d) {
		leftTalon1.set(ControlMode.PercentOutput, limit(d));
	}

	public void setRight(double d) {
		rightTalon1.set(ControlMode.PercentOutput, limit(d));
	}

	private double limit(double d) {
		return Util.limit(d, -1, 1);
	}

}