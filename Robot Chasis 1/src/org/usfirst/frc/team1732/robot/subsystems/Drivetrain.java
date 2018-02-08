package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	public TalonSRX leftTalon1;
	public TalonSRX rightTalon1;

	public DifferentialDrive drive;

	private final TalonEncoder leftEncoder;
	private final TalonEncoder rightEncoder;

	public static final double DRIVE_DEADBAND = 0.04; // CTRE default, but also need to pass to DifferentialDrive
	public static final int ENCODER_PULSES_PER_INCH = 520; // probably should double check this

	public Drivetrain() {

		int leftMaster = 1;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, TalonConfiguration.DEFAULT_CONFIG);
		// MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false,
		// TalonConfiguration.DEFAULT_CONFIG), leftTalon1);
		// MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false,
		// TalonConfiguration.DEFAULT_CONFIG), leftTalon1);

		VictorSPX test = new VictorSPX(leftMaster);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, TalonConfiguration.DEFAULT_CONFIG);
		// MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true,
		// TalonConfiguration.DEFAULT_CONFIG), rightTalon1);
		// MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true,
		// TalonConfiguration.DEFAULT_CONFIG), rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1);
		drive.setDeadband(DRIVE_DEADBAND); // might not need these: talon's have their own "neutral zone"
		leftEncoder = new TalonEncoder(leftTalon1, FeedbackDevice.QuadEncoder);
		rightEncoder = new TalonEncoder(rightTalon1, FeedbackDevice.QuadEncoder);
		leftEncoder.setDistancePerPulse(1.0 / ENCODER_PULSES_PER_INCH);
		rightEncoder.setDistancePerPulse(1.0 / ENCODER_PULSES_PER_INCH);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Velocity", leftEncoder.getRate());
		SmartDashboard.putNumber("Right Velocity", rightEncoder.getRate());
	}

	public EncoderReader getRightEncoderReader() {
		return rightEncoder.makeReader();
	}

	public EncoderReader getLeftEncoderReader() {
		return leftEncoder.makeReader();
	}

	public void setStop() {
		drive.tankDrive(0, 0);
	}

}