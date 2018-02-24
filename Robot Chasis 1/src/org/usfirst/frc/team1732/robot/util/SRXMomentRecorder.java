package org.usfirst.frc.team1732.robot.util;

import java.util.Deque;
import java.util.concurrent.LinkedBlockingDeque;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class SRXMomentRecorder {
	private TalonSRX talon;
	private TalonEncoder encoder;
	private Deque<Moment> moments = new LinkedBlockingDeque<>();
	private boolean recording = false;
	private Moment currentMoment = null;

	public SRXMomentRecorder(TalonSRX device, TalonEncoder reader) {
		talon = device;
		encoder = reader;
		moments.push(new Moment(0, 0, 0, 0, 0));
	}

	private double vel;
	private double time;
	private int i = 0;
	private double totalTime = 0;

	public void startRecording() {
		moments.clear();
		recording = true;
		time = Timer.getFPGATimestamp();
		vel = encoder.getRate();
		new Thread(this::runRecord).start();
	}

	private void runRecord() {
		while (!Thread.interrupted()) {
			record();
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
			}
		}
	}

	public void record() {
		if (recording) {
			moments.push(new Moment(talon.getMotorOutputVoltage(),
					talon.getMotorOutputPercent(), encoder.getRate(),
					(encoder.getRate() - vel) / (Timer.getFPGATimestamp() - time),
					Timer.getFPGATimestamp() - time));
			totalTime += Timer.getFPGATimestamp() - time;
			i++;
			vel = encoder.getRate();
			time = Timer.getFPGATimestamp();
		}
	}

	public void stopRecording() {
		recording = false;
		System.out.println("Average Time: " + totalTime / i +
				" vs theory: " + Robot.PERIOD_S);
	}

	public double getLastVoltage() {
		if (isFinished()) {
			currentMoment = null;
			return 0;
		} else {
			currentMoment = moments.pop();
			return currentMoment.velocity;
		}
	}

	public Moment getLast() {
		return moments.pop();
	}

	private Moment current;

	public Moment getNext(double deltaTime) {
		if(isFinished()) {
			return null;
		}
		if (current == null) {
			current = getLast();
			current.deltaTime -= deltaTime;
			return current;
		}

		if (current.deltaTime > deltaTime) {
			current.deltaTime-= deltaTime;
			return current;
		}else {
			deltaTime-= current.deltaTime;
			current = getLast();
			return getNext(deltaTime);
		}
	}

	public boolean isFinished() {
		return moments.isEmpty();
	}

	public class Moment {
		/**
		 * Voltage in <strong>volts</strong>
		 */
		public final double voltage;
		/**
		 * Percent Output
		 */
		public final double percent;
		/**
		 * Velocity in <strong>in / sec</strong>
		 */
		public final double velocity;
		/**
		 * Acceleration in <strong>in / sec<sup>2</sup></strong>
		 */
		public final double acceleration;
		/**
		 * The change in time in <strong>sec</strong>
		 */
		protected double deltaTime;

		public Moment(double v, double p, double c, double a, double t) {
			voltage = v;
			percent = p;
			velocity = c;
			acceleration = a;
			deltaTime = t;
		}

		public String toString() {
			return String.format("Voltage: %.3f, EncoderPos: %.3f", voltage, velocity);
		}
	}
}
