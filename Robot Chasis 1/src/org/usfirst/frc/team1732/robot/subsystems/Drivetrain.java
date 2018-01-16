package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.config.Node;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.TalonEncoder;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	public DifferentialDrive drive;

	private TalonSRX leftTalon1;
	private TalonSRX rightTalon1;

	public final TalonEncoder leftEncoder;
	public final TalonEncoder rightEncoder;

	public static final double DRIVE_DEADBAND = 0.04; // CTRE default, but also need to pass to DifferentialDrive

	public Drivetrain(Node drivetrainNode) {
		leftTalon1 = MotorUtils.configureTalon(drivetrainNode.getNode("leftTalon1"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(drivetrainNode.getNode("leftTalon2"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(drivetrainNode.getNode("leftTalon3"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		rightTalon1 = MotorUtils.configureTalon(drivetrainNode.getNode("rightTalon1"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(drivetrainNode.getNode("rightTalon2"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(drivetrainNode.getNode("rightTalon3"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		drive = new DifferentialDrive(leftTalon1, rightTalon1);
		drive.setDeadband(DRIVE_DEADBAND); // might not need these: talon's have their own "neutral zone"
		leftEncoder = new TalonEncoder(leftTalon1);
		rightEncoder = new TalonEncoder(rightTalon1);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Encoder Pulses", leftEncoder.getPulses());
		SmartDashboard.putNumber("Right Encoder Pulses", rightEncoder.getPulses());

	}
}