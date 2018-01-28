package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import java.awt.Point;
import java.nio.file.Path;

import org.usfirst.frc.team1732.robot.controlutils.GainProfile;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DrivetrainProfileManager {

	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	private MotionProfileManager leftManager;
	private MotionProfileManager rightManager;

	private final double sensorUnitsPerYourUnits;

	private final double robotWidth;

	public DrivetrainProfileManager(TalonSRX leftTalon,
			TalonSRX rightTalon, double sensorUnitsPerYourUnits, double robotWidth) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		this.sensorUnitsPerYourUnits = sensorUnitsPerYourUnits;
		this.robotWidth = robotWidth;
	}

	public void followNewPath(Point[] waypoints, double totalTimeSeconds,
			int stepDurationMilliseconds, GainProfile profile) {
		followNewPath(waypoints, totalTimeSeconds, stepDurationMilliseconds, profile, profile);
	}

	public void followNewPath(Point[] waypoints, double totalTimeSeconds,
			int stepDurationMilliseconds, GainProfile leftProfile,
			GainProfile rightProfile) {

		if (leftManager == null && rightManager == null) { // both should always be null
			leftManager = new MotionProfileManager(leftTalon, null);
			rightManager = new MotionProfileManager(rightTalon, null);

		} else {
			leftManager.resetAndChangePath(null);
			rightManager.resetAndChangePath(null);
		}
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during disabled
	 * and when Talon is not in MP control mode.
	 */
	public void reset() {
		leftManager.reset();
		rightManager.reset();
	}

	/**
	 * Called every robot loop while profile is running to keep talon in right mode
	 */
	public void run() {
		leftManager.run();
		rightManager.run();
	}

	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	public void startMotionProfile() {
		leftManager.startMotionProfile();
		rightManager.startMotionProfile();
	}

	/*
	 * stepDuration is in ms
	 */
	private static Path makePathFromFalconData(int stepDuration, double[][] positions, double[][] velocities,
			GainProfile profile, double sensorUnitPerYourUnits) {
		TrajectoryPoint[] points = new TrajectoryPoint[positions.length];
		points[0] = new TrajectoryPoint(0.0, velocities[0][1] * sensorUnitPerYourUnits / 10.0, stepDuration);

		for (int i = 1; i < positions.length; i++) {
			double position = (points[i - 1].position + velocities[i][1] * stepDuration / 1000.0)
					* sensorUnitPerYourUnits;
			double dx = positions[i][0] - positions[i - 1][0];
			double dy = positions[i][1] - positions[i - 1][1];
			double heading = Math.toDegrees(-Math.atan2(dx, dy) + Math.PI / 2.0);
			points[i] = new TrajectoryPoint(position, (double) (velocities[i][1] * sensorUnitPerYourUnits / 10.0),
					stepDuration);
		}
		return null;
	}

}