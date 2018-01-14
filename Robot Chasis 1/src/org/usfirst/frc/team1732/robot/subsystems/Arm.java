package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.config.Node;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {

	public final TalonSRX talon;

	public Arm(Node armNode) {
		talon = MotorUtils.configureTalon(armNode.getNode("talon"), 0, Robot.CONFIG_TIMEOUT);
	}

	public void setUp() {
		setSpeed(0.5);
	}

	public void setDown() {
		setSpeed(-0.5);
	}

	public void setStop() {
		setSpeed(0.0);
	}

	public void setSpeed(double speed) {
		talon.set(ControlMode.PercentOutput, speed);
	}

	@Override
	protected void initDefaultCommand() {

	}

}
