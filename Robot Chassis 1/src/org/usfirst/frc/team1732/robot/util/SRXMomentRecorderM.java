package org.usfirst.frc.team1732.robot.util;

import java.util.Deque;
import java.util.concurrent.LinkedBlockingDeque;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class SRXMomentRecorderM {
	private TalonSRX left;
	private TalonEncoder leftr;
	private TalonSRX right;
	private TalonEncoder rightr;
	private Deque<Pair<Moment>> moments = new LinkedBlockingDeque<>();
	private boolean recording = false;
	private Moment currentMoment = null;
	
	public SRXMomentRecorderM(TalonSRX left, TalonEncoder leftr, TalonSRX right, TalonEncoder rightr) {
		this.left = left;
		this.leftr = leftr;
		this.right = right;
		this.rightr = rightr;
	}

	private double lvel;
	private double rvel;
	private double time;
	private int i = 0;
	private double totalTime = 0;
	
	public void startRecording() {
		moments.clear();
		recording = true;
		time = Timer.getFPGATimestamp();
		rvel = rightr.getRate();
		lvel = leftr.getRate();
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
			moments.push(new Pair<Moment>(
					new Moment(left.getMotorOutputVoltage(),
					left.getMotorOutputPercent(), leftr.getRate(),
					(leftr.getRate() - lvel) / (Timer.getFPGATimestamp() - time),
					Robot.sensors.navX.getTotalAngle(), Timer.getFPGATimestamp() - time),
					new Moment(right.getMotorOutputVoltage(),
					right.getMotorOutputPercent(), rightr.getRate(),
					(rightr.getRate() - rvel) / (Timer.getFPGATimestamp() - time),
					Robot.sensors.navX.getTotalAngle(), Timer.getFPGATimestamp() - time)));
			totalTime += Timer.getFPGATimestamp() - time;
			i++;
			lvel = leftr.getRate();
			rvel = rightr.getRate();
			time = Timer.getFPGATimestamp();
		}
	}
	
	public void stopRecording() {
		recording = false;
		System.out.println("Average Time: " + totalTime / i + " vs theory: " + Robot.PERIOD_S);
	}
	
	public Pair<Moment> getLast() {
		return moments.pop();
	}
	
	private Pair<Moment> current;
	
	public Pair<Moment> getNext(double deltaTime) {
		if (isFinished()) {
			return null;
		}
		if (current == null) {
			current = getLast();
			current.one.deltaTime -= deltaTime;
			return current;
		}
		
		if (current.one.deltaTime > deltaTime) {
			current.one.deltaTime -= deltaTime;
			return current;
		} else {
			deltaTime -= current.one.deltaTime;
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
		 * 
		 */
		public final double heading;
		/**
		 * The change in time in <strong>sec</strong>
		 */
		protected double deltaTime;
		
		public Moment(double voltage, double percent, double velocity, double acceleration,
				double heading, double deltaTime) {
			this.voltage = voltage;
			this.percent = percent;
			this.velocity = velocity;
			this.acceleration = acceleration;
			this.heading = heading;
			this.deltaTime = deltaTime;
		}
		
		@Override
		public String toString() {
			return String.format("Voltage: %.3f, EncoderPos: %.3f", voltage, velocity);
		}
	}
}
