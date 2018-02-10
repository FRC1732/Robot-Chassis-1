package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.controlutils.Feedforward;
import org.usfirst.frc.team1732.robot.controlutils.GainProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.DoubleProfileManager;
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
	private static final double fix = 79.5 / 100.0;
	public static final double ENCODER_INCHES_PER_PULSE = 0.002099 * fix;

	public static final double PERCENT_BASE_VOLTAGE = 0.9;
	public final Feedforward leftFFF = new Feedforward(0.063329 / fix, 0.010514 / fix, 1.395889*PERCENT_BASE_VOLTAGE);
	public final Feedforward leftBFF = new Feedforward(0.062512 / fix, 010545 / fix, -1.407502*PERCENT_BASE_VOLTAGE);
	public final Feedforward rightFFF = new Feedforward(0.062081 / fix, 0.010137 / fix, 1.486594*PERCENT_BASE_VOLTAGE);
	public final Feedforward rightBFF = new Feedforward(0.062407 / fix, 0.010243 / fix, -1.465781*PERCENT_BASE_VOLTAGE);

	public final GainProfile leftGains = new GainProfile("Left PID", 0.0, 0, 0, leftFFF, 0, 0, 0);
	public final GainProfile rightGains = new GainProfile("Right PID", 0.0, 0, 0, rightFFF, 0, 0, 0);

	public static final double MAX_IN_SEC = 84;
	public static final double MAX_IN_SEC2 = 250;

	public final DoubleProfileManager profileManager;

	public Drivetrain() {
		int leftMaster = 1;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, TalonConfiguration.DEFAULT_CONFIG);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false, TalonConfiguration.DEFAULT_CONFIG),
				leftTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false, TalonConfiguration.DEFAULT_CONFIG),
				leftTalon1);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, TalonConfiguration.DEFAULT_CONFIG);

		MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true, TalonConfiguration.DEFAULT_CONFIG),
				rightTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true, TalonConfiguration.DEFAULT_CONFIG),
				rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1, ControlMode.PercentOutput, MIN_OUTPUT, MAX_OUTPUT,
				INPUT_DEADBAND);

		leftEncoder = new TalonEncoder(leftTalon1, FeedbackDevice.QuadEncoder);
		rightEncoder = new TalonEncoder(rightTalon1, FeedbackDevice.QuadEncoder);
		leftEncoder.setPhase(true);
		rightEncoder.setPhase(true);
		leftEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.zero();
		leftEncoder.zero();
		profileManager = new DoubleProfileManager(leftTalon1, rightTalon1);
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