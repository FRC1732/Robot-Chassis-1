package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.config.Node;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OtherMotors {

	public final TalonSRX otherTalon1;
	// public final TalonSRX otherTalon2;

	public OtherMotors(Node otherMotorsNode) {
		otherTalon1 = MotorUtils.configureTalon(otherMotorsNode.getNode("otherTalon1"),
				Drivetrain.DRIVE_DEADBAND, Robot.CONFIG_TIMEOUT);
		// otherTalon2 =
		// MotorUtils.configureTalon(otherMotorsNode.getNode("otherTalon2"),
		// Drivetrain.DRIVE_DEADBAND, Robot.CONFIG_TIMEOUT);
		SmartDashboard.putNumber("Other Motor 1", 0);
		// SmartDashboard.putNumber("Other Motor 2", 0);
	}

	public void run() {
		otherTalon1.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Other Motor 1", 0));
		// otherTalon2.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Other
		// Motor 2", 0));
	}

}