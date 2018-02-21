package org.usfirst.frc.team1732.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;

public class SampleAveraging {
	
	private double total;
	private int samples;
	
	private final Supplier<Double> s;
	private final Notifier n;
	private final double period;
	
	/**
	 * Takes samples no less than every `samples` miliseconds
	 * @param samples the number of samples to take
	 */
	public SampleAveraging(int sampleMillis, Supplier<Double> s) {
		this.s = s;
		period = sampleMillis / 1000.0;
		
		n = new Notifier(this::add);
		n.startPeriodic(period);
	}
	
	public void stop() {
		n.stop();
	}
	public void start() {
		n.startPeriodic(period);
	}
	
	private void add() {
		samples++;
		total+= s.get();
	}
	
	public double getAverage() {
		return total / samples;
	}

}
