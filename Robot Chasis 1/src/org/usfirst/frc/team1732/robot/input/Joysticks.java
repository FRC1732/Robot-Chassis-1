package org.usfirst.frc.team1732.robot.input;

import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.w3c.dom.Element;

import edu.wpi.first.wpilibj.Joystick;

public class Joysticks {

	public final Joystick left;
	public final Joystick right;

	public Joysticks(Element joysticks) {
		left = new Joystick(ConfigUtils.getInteger(ConfigUtils.getElement(joysticks, "left"), "port"));
		right = new Joystick(ConfigUtils.getInteger(ConfigUtils.getElement(joysticks, "right"), "port"));
	}
}