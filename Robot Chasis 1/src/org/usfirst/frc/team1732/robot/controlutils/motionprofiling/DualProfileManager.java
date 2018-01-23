package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import org.usfirst.frc.team1732.robot.controlutils.pathing.Path;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DualProfileManager {

	private final Path leftPath;
	private final Path rightPath;

	private final MotionProfileManager leftManager;
	private final MotionProfileManager rightManager;

	public DualProfileManager(Path leftPath, Path rightPath, TalonSRX left, TalonSRX right) {
		this.leftPath = leftPath;
		this.rightPath = rightPath;
		this.leftManager = new MotionProfileManager(right, leftPath);
		this.rightManager = new MotionProfileManager(right, rightPath);
	}

}