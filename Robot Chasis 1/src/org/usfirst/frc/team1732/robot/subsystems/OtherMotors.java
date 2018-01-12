package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.w3c.dom.Element;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class OtherMotors {

	public final TalonSRX otherMotor1;
	public final TalonSRX otherMotor2;

	public OtherMotors(Element otherMotors) {
		otherMotor1 = MotorUtils.configureTalon(ConfigUtils.getElement(otherMotors, "otherMotor1"),
				Drivetrain.DRIVE_DEADBAND, Drivetrain.CONFIG_TIMEOUT);
		otherMotor2 = MotorUtils.configureTalon(ConfigUtils.getElement(otherMotors, "otherMotor2"),
				Drivetrain.DRIVE_DEADBAND, Drivetrain.CONFIG_TIMEOUT);
	}

}