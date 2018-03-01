package org.usfirst.frc.team1732.robot.util;

import java.util.TreeMap;

import org.usfirst.frc.team1732.robot.Util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SRXVoltageRecord {
	public static final double VOLTAGE_COMPENSATION_SATURATION = 12;
	private TalonSRX talon;
	private TreeMap<Double, Double> voltages;

	public SRXVoltageRecord(TalonSRX device) {
		talon = device;
		voltages = new TreeMap<>();
	}

	public void clear() {
		voltages.clear();
	}

	public void addVoltage(double time) {
		// double volt = talon.getMotorOutputVoltage();
		voltages.put(time, talon.getMotorOutputPercent());
	}

	public double getVoltageAtTime(double time) {
		return Util.interpolateFromTree(voltages, time) * VOLTAGE_COMPENSATION_SATURATION;
	}

	public double getPercentOutputAtTime(double time) {
		return Util.interpolateFromTree(voltages, time);
	}

	public double getTimeLength() {
		return voltages.lastKey();
	}

	public double getLastPercent() {
		return voltages.lastEntry().getValue();
	}

}
