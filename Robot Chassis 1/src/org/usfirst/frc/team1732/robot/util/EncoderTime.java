package org.usfirst.frc.team1732.robot.util;

import java.util.Stack;

import org.usfirst.frc.team1732.robot.Robot;

public class EncoderTime {
	public final double leftSpeed;
	public final double rightSpeed;
	public final int deltaMS;

	public EncoderTime(double leftSpeed, double rightSpeed, int deltaMS) {
		this.leftSpeed = leftSpeed;
		this.rightSpeed = rightSpeed;
		this.deltaMS = deltaMS;
	}

	private static Stack<EncoderTime> times = new Stack<>();
	private static boolean recording = false;

	public static void startRecording() {
		times.clear();
		recording = true;
		new Thread(EncoderTime::record).start();
	}

	private static void record() {
		while (recording) {
			times.push(new EncoderTime(Robot.drivetrain.leftEncoder.getRate(), Robot.drivetrain.rightEncoder.getRate(),
					0));
			try {
				Thread.sleep(Robot.PERIOD_MS);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
