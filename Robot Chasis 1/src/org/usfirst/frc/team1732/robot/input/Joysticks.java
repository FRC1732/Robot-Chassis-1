package org.usfirst.frc.team1732.robot.input;

import org.usfirst.frc.team1732.robot.commands.RunClawIn;
import org.usfirst.frc.team1732.robot.commands.RunClawOut;
import org.usfirst.frc.team1732.robot.commands.armcontrol.SetArmIntake;
import org.usfirst.frc.team1732.robot.commands.armcontrol.SetArmScore;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Joysticks {

	public static final int LEFT_PORT = 0;
	public static final int RIGHT_PORT = 1;
	public static final int BUTTONS_PORT = 2;

	public final Joystick left;
	public final Joystick right;
	public final Joystick buttons;

	public Joysticks() {
		left = new Joystick(LEFT_PORT);
		right = new Joystick(RIGHT_PORT);
		buttons = new Joystick(BUTTONS_PORT);

		new JoystickButton(left, 1).whileHeld(new RunClawIn());

		// new JoystickButton(left, 1).whileHeld(new RunArmWithTorqueCompensation(0));

		new JoystickButton(right, 1).whileHeld(new RunClawOut());
		//
		new JoystickButton(left, 2).whileHeld(new SetArmScore());
		new JoystickButton(right, 2).whileHeld(new SetArmIntake());
	}

	// our joysticks return negative when they're pressed forward

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