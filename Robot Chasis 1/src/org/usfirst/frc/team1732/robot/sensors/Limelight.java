package org.usfirst.frc.team1732.robot.sensors;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
	private static final String LED_MODE = "ledMode", CAM_MODE = "camMode";
	private static final int BUFFER_SIZE = 10;

	private final NetworkTable table;

	private double horizontalOffsetAverage;

	public Limelight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		table.getEntry("tx").addListener(new Consumer<EntryNotification>() {
			public void accept(EntryNotification t) {
				// calculate rolling average
				horizontalOffsetAverage -= horizontalOffsetAverage / BUFFER_SIZE;
				horizontalOffsetAverage += t.value.getDouble() / BUFFER_SIZE;
			}
		}, EntryListenerFlags.kUpdate);
	}
	// ----- SETTERS -----
	public void setLEDMode(LEDMode mode) {
		table.getEntry(LED_MODE).setNumber(mode.getVal());
	}
	public void setCamMode(CamMode mode) {
		table.getEntry(CAM_MODE).setNumber(mode.getVal());
	}
	// ----- GETTERS -----
	// returns the horizonatal offset of the target (between -27 and 27 degrees)
	public double getRawHorizontalOffset() {
		return table.getEntry("tx").getNumber(0).doubleValue();
	}
	public double getRawHorizontalOffset(double defaultValue) {
		return hasValidTargets() ? getRawHorizontalOffset() : defaultValue;
	}
	// returns horizontal offset between 0 and 1
	public double getNormalizedHorizontalOffset() {
		return getRawHorizontalOffset() / 27;
	}
	public double getNormalizedHorizontalOffset(double defaultValue) {
		return hasValidTargets() ? getNormalizedHorizontalOffset() : defaultValue;
	}
	// returns rolling-averaged horizontal offset
	public double getHorizontalOffset() {
		return horizontalOffsetAverage;
	}
	public double getHorizontalOffset(double defaultValue) {
		return hasValidTargets() ? getHorizontalOffset() : defaultValue;
	}
	// returns the vertical offset of the target (between -20.5 and 20.5 degrees)
	public double getVerticalOffset() {
		return table.getEntry("ty").getNumber(0).doubleValue();
	}
	public double getVerticalOffset(double defaultValue) {
		return hasValidTargets() ? getVerticalOffset() : defaultValue;
	}
	// returns true if it is tracking a target
	public boolean hasValidTargets() {
		return table.getEntry("tv").getNumber(0).intValue() == 1;
	}
	// returns percent of screen area target takes up (between 0 and 100)
	public double getTargetArea() {
		return table.getEntry("ta").getNumber(0).doubleValue();
	}
	public double getTargetArea(double defaultValue) {
		return hasValidTargets() ? getTargetArea() : defaultValue;
	}
	// returns target area between 0 and 1
	public double getNormalizedTargetArea() {
		return getTargetArea() / 100;
	}
	// Estimates distance in inches using target area
	public double getDistanceToTarget(double defaultValue) {
		return hasValidTargets() ? 123.31 * Math.pow(getTargetArea(), -0.529) : defaultValue;
	}
	// GATHERED DATA
	/*
	 * AREA____DIST
	 * 1.0______118
	 * 1.2______108
	 * 1.5_______96
	 * 2.3_______84
	 * 3.0_______72
	 * 4.3_______60
	 * 5.8_______48
	 * 10.2______36
	 * 23.2______24
	 * 88.1______12
	 * 100.0_____10
	 */
	// rotation of target (between -90 and 0 degrees)
	public double getTargetSkew() {
		return table.getEntry("ts").getNumber(0).doubleValue();
	}
	// returns pipeline latency in milliseconds
	// doesnt take image capture latency into account (website says add 11 ms)
	public double getLatency() {
		return table.getEntry("tl").getNumber(0).doubleValue();
	}

	public void toggleLED() {
		NetworkTableEntry lights = table.getEntry(LED_MODE);
		lights.setNumber(lights.getNumber(1).intValue() == 0 ? 1 : 0);
	}
	public void toggleCamMode() {
		NetworkTableEntry cam = table.getEntry(CAM_MODE);
		cam.setNumber(cam.getNumber(1).intValue() == 0 ? 1 : 0);
	}

	public static enum LEDMode {
		ON(0), OFF(1), BLINK(2);
		private int val;

		private LEDMode(int n) {
			val = n;
		}
		public int getVal() {
			return val;
		}
	}
	public static enum CamMode {
		VISION_PROCESSOR(0), DRIVER_FEEDBACK(1);
		private int val;

		private CamMode(int n) {
			val = n;
		}
		public int getVal() {
			return val;
		}
	}
}
