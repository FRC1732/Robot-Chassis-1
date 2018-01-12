package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.w3c.dom.Element;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OtherMotors {

	public final TalonSRX otherMotor1;
	public final TalonSRX otherMotor2;

	public OtherMotors(Element otherMotors) {
		otherMotor1 = MotorUtils.configureTalon(ConfigUtils.getElement(otherMotors, "otherMotor1"),
				Drivetrain.DRIVE_DEADBAND, Robot.CONFIG_TIMEOUT);
		otherMotor2 = MotorUtils.configureTalon(ConfigUtils.getElement(otherMotors, "otherMotor2"),
				Drivetrain.DRIVE_DEADBAND, Robot.CONFIG_TIMEOUT);
		SmartDashboard.putNumber("Other Motor 1", 0);
		SmartDashboard.putNumber("Other Motor 2", 0);
	}

	public void run() {
		otherMotor1.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Other Motor 1", 0));
		otherMotor2.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Other Motor 2", 0));
	}

}