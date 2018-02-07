package org.usfirst.frc.team1732.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

// configure base and open loop parameters. Configure close loop parameters somewhere else

public class TalonConfiguration {

	public static final TalonConfiguration DEFAULT_CONFIG = new TalonConfiguration();

	public static final int CONFIG_TIMEOUT = 10;

	public NeutralMode neutralMode = NeutralMode.Coast;
	public double neutralDeadbandPercent = 0.0;
	public double nominalForward = +0.0;
	public double nominalReverse = -0.0;
	public double peakOutputForward = +1.0;
	public double peakOutputReverse = -1.0;
	public double openLoopRamp = 0; // 0 means disabled. This number means seconds from neutral to full throttle
	public boolean enableVoltageCompensation = false;
	public double voltageCompensationSaturation = 12;
	public boolean enableCurrentLimit = false;
	public int continousCurrentLimit = 0;
	public int peakCurrentLimit = 0;
	public int peakCurrentDuration = 0;
	public VelocityMeasPeriod velocityMeasurementPeriod = VelocityMeasPeriod.Period_10Ms;
	public int velocityMeasurementWindow = 4;
	public int voltageMeasurementWindow = 4;

	// update rate from talon in ms
	// these are the defaults. In general, don't change them.
	public int generalStatusPeriod = 10;
	public int feedback0StatusPeriod = 20;
	// don't change the following if you're not using other sensors, instead use
	// setSelectedSensor();
	public int quadratureStatusPeriod = 160;
	public int analogTemperatureBatteryStatusPeriod = 160;
	public int pulseWidthStatusPeriod = 160;

	public int currentMPtargetStatusPeriod = 160;
	public int PIDerrorStatusPeriod = 160;
	public int controlFramePeriod = 10;

	// ControlFrame.Control_3_General;
	// ControlFrame.Control_4_Advanced;
	// ControlFrame.Control_6_MotProfAddTrajPoint;
	// StatusFrame.Status_1_General;
	// StatusFrame.Status_2_Feedback0;
	// StatusFrame.Status_4_AinTempVbat;
	// StatusFrame.Status_6_Misc;
	// StatusFrame.Status_7_CommStatus;
	// StatusFrame.Status_9_MotProfBuffer;
	// StatusFrame.Status_10_MotionMagic;
	// StatusFrame.Status_12_Feedback1;
	// StatusFrame.Status_13_Base_PIDF0;
	// StatusFrame.Status_14_Turn_PIDF1;
	// StatusFrame.Status_15_FirmwareApiStatus;
	// StatusFrameEnhanced.Status_1_General;
	// StatusFrameEnhanced.Status_2_Feedback0;
	// StatusFrameEnhanced.Status_3_Quadrature;
	// StatusFrameEnhanced.Status_4_AinTempVbat;
	// StatusFrameEnhanced.Status_6_Misc;
	// StatusFrameEnhanced.Status_7_CommStatus;
	// StatusFrameEnhanced.Status_8_PulseWidth;
	// StatusFrameEnhanced.Status_9_MotProfBuffer;
	// StatusFrameEnhanced.Status_10_MotionMagic;
	// StatusFrameEnhanced.Status_11_UartGadgeteer;
	// StatusFrameEnhanced.Status_12_Feedback1;
	// StatusFrameEnhanced.Status_13_Base_PIDF0;
	// StatusFrameEnhanced.Status_14_Turn_PIDF1;
	// StatusFrameEnhanced.Status_15_FirmareApiStatus;
}
