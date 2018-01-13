package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.TalonEncoder;
import org.w3c.dom.Element;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

	public DifferentialDrive drive;

	private TalonSRX leftTalon1;
	private TalonSRX rightTalon1;

	public final TalonEncoder leftEncoder;
	public final TalonEncoder rightEncoder;

	public static final double DRIVE_DEADBAND = 0.04; // CTRE default, but also need to pass to DifferentialDrive

	public Drivetrain(Element driveTrain) {
		leftTalon1 = MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon1"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon2"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon3"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		rightTalon1 = MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon1"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon2"), DRIVE_DEADBAND,
				Robot.CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon3"), DRIVE_DEADBAND,
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

	}
}