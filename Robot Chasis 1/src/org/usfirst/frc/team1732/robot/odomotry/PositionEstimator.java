package org.usfirst.frc.team1732.robot.odomotry;

import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PositionEstimator extends Subsystem {

	private final AHRS navX;
	private final EncoderReader leftEncoder;
	private final EncoderReader rightEncoder;

	/*
	 * These are called relative because they are relative to robot start state in
	 * world reference frame (x, y, and heading)
	 */
	private double relativeX;
	private double relativeY;
	private double relativeHeading;

	public PositionEstimator(AHRS navX, EncoderReader leftEncoder, EncoderReader rightEncoder) {
		this.navX = navX;
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
	}

	public void reset() {
		relativeX = 0;
		relativeY = 0;
		relativeHeading = 0;
	}

	public void setPosition(double x, double y, double heading) {
		relativeX = x;
		relativeY = y;
		relativeHeading = heading;
	}

	private long lastLoop = System.currentTimeMillis();

	private void update() {
		// multiply by 100 because it's in units per 100 ms
		double velocity = (leftEncoder.getRate() * 100 + rightEncoder.getRate() * 100) / 2;
		relativeHeading = navX.getYaw();
		double headingRadians = Math.toRadians(relativeHeading);
		long dt = System.currentTimeMillis() - lastLoop;
		relativeX += Math.cos(headingRadians) + velocity * dt;
		relativeY += Math.sin(headingRadians) + velocity * dt;
		lastLoop = System.currentTimeMillis();
	}

	@Override
	public void periodic() {
		update();
		SmartDashboard.putNumber("Heading", getRelativeHeading());
		SmartDashboard.putNumber("X", getRelativeX());
		SmartDashboard.putNumber("Y", getRelativeY());
	}

	public double getRelativeX() {
		return relativeX;
	}

	public double getRelativeY() {
		return relativeY;
	}

	public double getRelativeHeading() {
		return relativeHeading;
	}

	@Override
	protected void initDefaultCommand() {

	}

}