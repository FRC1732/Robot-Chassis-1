package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

	public TalonSRX leftTalon1;
	public TalonSRX rightTalon1;

	public DifferentialDrive drive;

	private final TalonEncoder leftEncoder;
	private final TalonEncoder rightEncoder;

	public static final double DRIVE_DEADBAND = 0.04; // CTRE default, but also need to pass to DifferentialDrive
	public static final int ENCODER_PULSES_PER_INCH = 520; // probably should double check this

	public Drivetrain() {
		int leftMaster = 0;
		leftTalon1 = MotorUtils.configTalon(leftMaster, true, TalonConfiguration.DEFAULT_CONFIG);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, true, TalonConfiguration.DEFAULT_CONFIG), leftMaster);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, true, TalonConfiguration.DEFAULT_CONFIG), leftMaster);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, false, TalonConfiguration.DEFAULT_CONFIG);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, false, TalonConfiguration.DEFAULT_CONFIG),
				rightMaster);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, false, TalonConfiguration.DEFAULT_CONFIG),
				rightMaster);

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