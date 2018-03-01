package org.usfirst.frc.team1732.robot.util;

import java.util.Stack;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDController;

public class SRXMomentRecorderD {
	private TalonSRX talon;
	private EncoderReader encoder;
	private Stack<Moment> moments;
	private boolean recording = false;
	private PIDController pid;
	private Moment currentMoment = null;

	public SRXMomentRecorderD(TalonSRX device, EncoderReader reader) {
		talon = device;
		encoder = reader;
		moments = new Stack<>();
		pid = new PIDController(0.01, 0, 0, new DisplacementPIDSource() {
			public double pidGet() {
				return currentMoment == null ? 0 : -(currentMoment.encoderPos - encoder.getPosition());
			}
		}, d -> {
		});
	}

	public void startRecording() {
		moments.clear();
		recording = true;
		new Thread(() -> {
			while (recording && !Thread.interrupted()) {
				moments.push(new Moment(talon.getMotorOutputPercent(), encoder.getPosition()));
				try {
					// no idea what this does, but it has to be there to make the wait work
					synchronized (this) {
						this.wait((long) Robot.PERIOD_MS);
					}
				} catch (InterruptedException e) {
				}
			}
		}).start();
	}

	public void stopRecording() {
		recording = false;
		pid.enable();
	}

	public double getLastVoltage() {
		if (isFinished()) {
			pid.disable();
			currentMoment = null;
			return 0;
		} else {
			currentMoment = moments.pop();
			double pidGet = pid.get(), out = currentMoment.voltage + pidGet;
			// System.out.printf("%s, CurrentEncoder: %.3f, PID: %.3f, Output: %.3f%n",
			// currentMoment.toString(),
			// encoder.getPosition(), pidGet, out);
			return out;
		}
	}

	public boolean isFinished() {
		return moments.empty();
	}

	private class Moment {
		public final double voltage;
		public final double encoderPos;

		public Moment(double v, double c) {
			voltage = v;
			encoderPos = c;
		}

		public String toString() {
			return String.format("Voltage: %.3f, EncoderPos: %.3f", voltage, encoderPos);
		}
	}
}
