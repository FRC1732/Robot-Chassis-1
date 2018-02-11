package org.usfirst.frc.team1732.robot.subsystems;

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
	// private static final double fix = 79.5 / 100.0;
	public static final double ENCODER_INCHES_PER_PULSE = 0.002099;

	// public static final double PERCENT_BASE_VOLTAGE = 1;

	public final Feedforward leftFFF = new Feedforward(0.063329, 0.010514, 1.395889);
	public final Feedforward leftBFF = new Feedforward(0.062512, 010545, -1.407502);
	public final Feedforward rightFFF = new Feedforward(0.062081, 0.010137, 1.486594);
	public final Feedforward rightBFF = new Feedforward(0.062407, 0.010243, -1.465781);

	public final ClosedLoopProfile leftGain = new ClosedLoopProfile("Left PID", 0.0, 0, 0,
			Feedforward.TALON_SRX_FF_GAIN, 0, 0, 0, 0);
	public final ClosedLoopProfile rightGain = new ClosedLoopProfile("Right PID", 0.0, 0, 0,
			Feedforward.TALON_SRX_FF_GAIN, 0, 0, 0, 0);

	public static final double MAX_IN_SEC = 84;
	public static final double MAX_IN_SEC2 = 250;

	public final DoubleProfileLoader profileManager;

	public Drivetrain() {
		int leftMaster = 1;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, TalonConfiguration.DEFAULT_CONFIG);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false, TalonConfiguration.DEFAULT_CONFIG), leftTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false, TalonConfiguration.DEFAULT_CONFIG), leftTalon1);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, TalonConfiguration.DEFAULT_CONFIG);

		MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true, TalonConfiguration.DEFAULT_CONFIG), rightTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true, TalonConfiguration.DEFAULT_CONFIG), rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1, ControlMode.PercentOutput, MIN_OUTPUT, MAX_OUTPUT,
				INPUT_DEADBAND);

		leftGain.applyToTalon(leftTalon1, 0, 0);
		rightGain.applyToTalon(rightTalon1, 0, 0);
		ClosedLoopProfile.applyZeroGainToTalon(leftTalon1, 0, 1);
		ClosedLoopProfile.applyZeroGainToTalon(rightTalon1, 0, 1);

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
		return rightEncoder.makeReader();
	}

	public EncoderReader makeLeftEncoderReader() {
		return leftEncoder.makeReader();
	}

	public void setStop() {
		drive.tankDrive(0, 0);
	}

}