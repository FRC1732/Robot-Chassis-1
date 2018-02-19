package org.usfirst.frc.team1732.robot.util;

import java.util.Stack;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

public class SRXMomentRecorder {
	private TalonSRX talon;
	private TalonEncoder encoder;
	private Stack<Moment> moments;
	private boolean recording = false;
	private Moment currentMoment = null;

	public SRXMomentRecorder(TalonSRX device, TalonEncoder reader) {
		talon = device;
		encoder = reader;
		moments = new Stack<>();
	}

	private double vel = 0;
	private long time = 0;

	public void startRecording() {
		moments.clear();
		recording = true;
		time = System.currentTimeMillis();
		vel = encoder.getRate();
		new Thread(() -> {
			while (recording && !Thread.interrupted()) {
				moments.push(new Moment(talon.getMotorOutputPercent(), encoder.getRate(),
						(encoder.getRate() - vel)/((System.currentTimeMillis() - time)/100)));
				time = System.currentTimeMillis();
				vel = encoder.getRate();
				try {
					// no idea what this does, but it has to be there to make the wait work
					synchronized (this) {
						Thread.sleep(Robot.PERIOD_MS);
					}
				} catch (InterruptedException e) {}
			}
		}).start();
	}

	public void stopRecording() {
		recording = false;
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

	public boolean isFinished() {
		return moments.empty();
	}

	public class Moment {
		public final double voltage;
		public final double velocity;
		public final double acceleration;

		public Moment(double v, double c, double a) {
			voltage = v;
			velocity = c;
			acceleration = a;

		}

		public String toString() {
			return String.format("Voltage: %.3f, EncoderPos: %.3f", voltage, velocity);
		}
	}
}
