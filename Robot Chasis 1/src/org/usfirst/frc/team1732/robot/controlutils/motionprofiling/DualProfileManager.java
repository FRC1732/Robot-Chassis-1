package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import org.usfirst.frc.team1732.robot.controlutils.pathing.Path;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DualProfileManager {

	private final Path path;
	private final MotionProfileManager leftManager;
	private final MotionProfileManager rightManager;

	public DualProfileManager(Path path, TalonSRX left, TalonSRX right) {
		this.path = path;
		this.leftManager = new MotionProfileManager(right, path.leftPoints);
		this.rightManager = new MotionProfileManager(right, path.rightPoints);
	}
}