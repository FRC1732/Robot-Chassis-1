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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	public final TalonSRX leftTalon1, l2, l3;
	public final TalonSRX rightTalon1, r2, r3;

	public DifferentialDrive drive;

	public final TalonEncoder leftEncoder;
	public final TalonEncoder rightEncoder;

	public static final double INPUT_DEADBAND = 0.025; // 2.5%.
	public static final double MIN_OUTPUT = 0.0;
	public static final double MAX_OUTPUT = 1.0;
	public static final double ENCODER_INCHES_PER_PULSE = 0.002099;

	public static final double MAX_IN_SEC = 150; // max vel
	public static final double MAX_UNITS_PER_100MS = MAX_IN_SEC / ENCODER_INCHES_PER_PULSE / 10; // max vel
	public static final double MAX_IN_SEC2 = 400; // max acc

	// Feedforward
	public final Feedforward leftFF = new Feedforward(0.0574657, 0.0089855, 1.6010019, 0.0574657, 0.0089855,
			-1.6010019);
	public final Feedforward rightFF = new Feedforward(0.0560625, 0.0071341, 1.7711541, 0.0560625, 0.0071341,
			-1.7711541);

	private final ClosedLoopProfile mpGains = new ClosedLoopProfile("MP PID", 0.4, 0, 0, Feedforward.TALON_SRX_FF_GAIN,
			0, 0, 0, 0);
	public final ClosedLoopProfile leftMPGains = mpGains;
	public final ClosedLoopProfile rightMPGains = mpGains;

	public final ClosedLoopProfile velocityGains = new ClosedLoopProfile("Velocity pid", 0.3, 0, 0,
			1023 / MAX_UNITS_PER_100MS, 0, 0, 0, 0);

	public static final double ROBOT_LENGTH_IN = 34.5;
	public static final double ROBOT_WIDTH_IN = 35;
	public static final double EFFECTIVE_ROBOT_WIDTH_IN = 27.5;

	public final DoubleProfileLoader profileManager;

	public Drivetrain() {
		int leftMaster = 1;
		TalonConfiguration config = TalonConfiguration.getDefaultConfig();
		config.enableVoltageCompensation = true;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, config);
		l2 = MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false, config), leftTalon1);
		l3 = MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false, config), leftTalon1);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, config);
		r2 = MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true, config), rightTalon1);
		r3 = MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true, config), rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1, ControlMode.PercentOutput, MIN_OUTPUT, MAX_OUTPUT,
				INPUT_DEADBAND);

		ClosedLoopProfile.applyZeroGainToTalon(leftTalon1, 0, 1);
		ClosedLoopProfile.applyZeroGainToTalon(rightTalon1, 0, 1);
		ClosedLoopProfile.applyZeroGainToTalon(leftTalon1, 1, 1);
		ClosedLoopProfile.applyZeroGainToTalon(rightTalon1, 1, 1);
		leftMPGains.applyToTalon(leftTalon1, 0, 0);
		rightMPGains.applyToTalon(rightTalon1, 0, 0);

		velocityGains.applyToTalon(leftTalon1, 1, 0);
		velocityGains.applyToTalon(rightTalon1, 1, 0);
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

	public double convertVelocitySetpoint(double desired) {
		return desired / MAX_IN_SEC * MAX_UNITS_PER_100MS;
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
		leftTalon1.neutralOutput();
		rightTalon1.neutralOutput();
	}

	public void setLeft(double d) {
		System.out.println("left:" + d);
		d = limit(d);
		leftTalon1.set(ControlMode.PercentOutput, d);
	}

	public void setRight(double d) {
		System.out.println("right: " + d);
		d = limit(d);
		rightTalon1.set(ControlMode.PercentOutput, limit(d));
	}

	private double limit(double d) {
		return Util.limit(d, -1, 1);
	}

	public void setNeutralMode(NeutralMode mode) {
		leftTalon1.setNeutralMode(mode);
		l2.setNeutralMode(mode);
		l3.setNeutralMode(mode);
		rightTalon1.setNeutralMode(mode);
		r2.setNeutralMode(mode);
		r3.setNeutralMode(mode);
	}

}