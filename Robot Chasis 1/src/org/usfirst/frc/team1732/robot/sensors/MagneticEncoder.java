package org.usfirst.frc.team1732.robot.sensors;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MagneticEncoder extends EncoderBase {

	private final TalonSRX talon;
	private double distancePerPulse;

	public MagneticEncoder(TalonSRX talon) {
		this.talon = talon;
		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, Robot.CONFIG_TIMEOUT);
		talon.configVelocityMeasurementWindow(2, Robot.CONFIG_TIMEOUT);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Robot.PERIOD_MS, Robot.CONFIG_TIMEOUT);

		// talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidIdx,
		// Robot.CONFIG_TIMEOUT);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, pidIdx,
		// timeoutMs);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, pidIdx,
		// timeoutMs);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, pidIdx,
		// timeoutMs);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.SoftwareEmulatedSensor,
		// pidIdx, timeoutMs);
	}

	public double getPosition() {
		return talon.getSensorCollection().getPulseWidthPosition() * distancePerPulse;

	}

	public double getRate() {
		return talon.getSensorCollection().getPulseWidthVelocity() * distancePerPulse;
	}

	public void setPhase(boolean phase) {
		talon.setSensorPhase(phase);
	}

	@Override
	public void setDistancePerPulse(double distancePerPulse) {
		this.distancePerPulse = distancePerPulse;
	}

	@Override
	public double getPulses() {
		return talon.getSensorCollection().getPulseWidthPosition();
	}

}