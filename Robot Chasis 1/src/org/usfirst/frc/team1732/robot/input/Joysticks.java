package org.usfirst.frc.team1732.robot.input;

import org.usfirst.frc.team1732.robot.commands.RunClawIn;
import org.usfirst.frc.team1732.robot.commands.RunClawOut;
import org.usfirst.frc.team1732.robot.commands.armcontrol.SetArmIntake;
import org.usfirst.frc.team1732.robot.commands.armcontrol.SetArmScore;
import org.usfirst.frc.team1732.robot.config.Node;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Joysticks {

	public final Joystick left;
	public final Joystick right;
	public final Joystick buttons;

	public Joysticks(Node joysticksNode) {
		left = new Joystick(joysticksNode.getNode("left").getData("port"));
		right = new Joystick(joysticksNode.getNode("right").getData("port"));
		buttons = new Joystick(joysticksNode.getNode("buttons").getData("port"));

		new JoystickButton(left, 1).whileHeld(new RunClawIn());

		// new JoystickButton(left, 1).whileHeld(new RunArmWithTorqueCompensation(0));

		new JoystickButton(right, 1).whileHeld(new RunClawOut());
		//
		new JoystickButton(left, 2).whileHeld(new SetArmScore());
		new JoystickButton(right, 2).whileHeld(new SetArmIntake());
	}

	public double getLeft() {
		return -left.getRawAxis(1);
	}

	public double getRight() {
		return -right.getRawAxis(1);
	}

	public boolean isReversed() {
		return buttons.getRawButton(5);
	}
}