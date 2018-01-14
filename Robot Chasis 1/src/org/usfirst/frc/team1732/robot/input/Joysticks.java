package org.usfirst.frc.team1732.robot.input;

import org.usfirst.frc.team1732.robot.commands.RunArmDown;
import org.usfirst.frc.team1732.robot.commands.RunArmUp;
import org.usfirst.frc.team1732.robot.commands.RunClawIn;
import org.usfirst.frc.team1732.robot.commands.RunClawOut;
import org.usfirst.frc.team1732.robot.config.Node;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Joysticks {

	public final Joystick left;
	public final Joystick right;

	public Joysticks(Node joysticksNode) {
		left = new Joystick(joysticksNode.getNode("left").getData("port"));
		right = new Joystick(joysticksNode.getNode("right").getData("port"));

		new JoystickButton(left, 1).whenPressed(new RunClawIn());
		new JoystickButton(right, 1).whenPressed(new RunClawOut());

		new JoystickButton(left, 2).whenPressed(new RunArmUp());
		new JoystickButton(right, 2).whenActive(new RunArmDown());
	}
}