package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.w3c.dom.Element;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

	public DifferentialDrive drive;

	private TalonSRX leftTalon1;
	private TalonSRX rightTalon1;

	public static final int CONFIG_TIMEOUT = 10; // recommended timeout by CTRE
	public static final double DRIVE_DEADBAND = 0.04; // CTRE default, but also need to pass to DifferentialDrive

	public Drivetrain(Element driveTrain) {
		leftTalon1 = MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon1"), DRIVE_DEADBAND,
				CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon2"), DRIVE_DEADBAND, CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "leftTalon3"), DRIVE_DEADBAND, CONFIG_TIMEOUT);
		rightTalon1 = MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon1"), DRIVE_DEADBAND,
				CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon2"), DRIVE_DEADBAND, CONFIG_TIMEOUT);
		MotorUtils.configureTalon(ConfigUtils.getElement(driveTrain, "rightTalon3"), DRIVE_DEADBAND, CONFIG_TIMEOUT);
		drive = new DifferentialDrive(leftTalon1, rightTalon1);
		drive.setDeadband(DRIVE_DEADBAND); // might not need these: talon's have their own "neutral zone"
	}

	@Override
	public void initDefaultCommand() {

	}

	@Override
	public void periodic() {

	}
}