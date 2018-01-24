package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import java.awt.Point;

import org.usfirst.frc.team1732.robot.controlutils.GainProfile;
import org.usfirst.frc.team1732.robot.controlutils.pathing.Path;
import org.usfirst.frc.team1732.robot.controlutils.pathing.TrajPoint;
import org.usfirst.frc.team1732.robot.controlutils.pathing.falcon.FalconPathPlanner;

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

		FalconPathPlanner planner = makeFalconPlanner(waypoints, totalTimeSeconds, stepDurationMilliseconds,
				robotWidth);
		Path leftPath = makePathFromFalconData(stepDurationMilliseconds, planner.leftPath, planner.smoothLeftVelocity,
				leftProfile, sensorUnitsPerYourUnits);
		Path rightPath = makePathFromFalconData(stepDurationMilliseconds, planner.rightPath,
				planner.smoothRightVelocity,
				rightProfile, sensorUnitsPerYourUnits);

		if (leftManager == null && rightManager == null) { // both should always be null
			leftManager = new MotionProfileManager(leftTalon, leftPath);
			rightManager = new MotionProfileManager(rightTalon, rightPath);
		} else {
			leftManager.resetAndChangePath(leftPath);
			rightManager.resetAndChangePath(rightPath);
		}
	}

	public void reset() {
		leftManager.reset();
		rightManager.reset();
	}

	public void run() {
		leftManager.run();
		rightManager.run();
	}

	public void startProfile() {
		leftManager.startMotionProfile();
		rightManager.startMotionProfile();
	}

	/*
	 * stepDuration is in ms
	 */
	private static Path makePathFromFalconData(int stepDuration, double[][] positions, double[][] velocities,
			GainProfile profile, double sensorUnitPerYourUnits) {
		TrajPoint[] points = new TrajPoint[positions.length];
		points[0] = new TrajPoint(0.0, velocities[0][1] * sensorUnitPerYourUnits / 10.0, stepDuration, 0.0);

		for (int i = 1; i < positions.length; i++) {
			double position = (points[i - 1].position + velocities[i][1] * stepDuration / 1000.0)
					* sensorUnitPerYourUnits;
			double dx = positions[i][0] - positions[i - 1][0];
			double dy = positions[i][1] - positions[i - 1][1];
			double heading = Math.toDegrees(-Math.atan2(dx, dy) + Math.PI / 2.0);
			points[i] = new TrajPoint(position, velocities[i][1] * sensorUnitPerYourUnits / 10.0, stepDuration,
					heading);
		}
		return new Path(points, profile, 0, stepDuration);
	}

	private static FalconPathPlanner makeFalconPlanner(Point[] waypoints, double totalTimeSeconds,
			int stepDurationMilliseconds,
			double robotWidth) {
		double[][] points = new double[waypoints.length][2];
		for (int i = 0; i < waypoints.length; i++) {
			points[i][0] = waypoints[i].getX();
			points[i][1] = waypoints[i].getY();
		}
		FalconPathPlanner planner = new FalconPathPlanner(points);
		planner.calculate(totalTimeSeconds, stepDurationMilliseconds / 1000.0, robotWidth);
		return planner;
	}

}