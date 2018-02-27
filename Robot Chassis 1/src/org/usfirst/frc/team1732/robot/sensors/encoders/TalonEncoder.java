package org.usfirst.frc.team1732.robot.sensors.encoders;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonEncoder extends EncoderBase {

	protected final TalonSRX talon;
	protected double distancePerPulse;

	public TalonEncoder(TalonSRX talon, FeedbackDevice selectedSensor) {
		this.talon = talon;
		talon.configSelectedFeedbackSensor(selectedSensor, 0, Robot.CONFIG_TIMEOUT);

		// talon.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs)
		// talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
		// Robot.PERIOD_MS, Robot.CONFIG_TIMEOUT);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, pidIdx,
		// timeoutMs);
		// talon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, pidIdx,
		// timeoutMs);
	}

	@Override
	public double getPosition() {
		return talon.getSelectedSensorPosition(0) * distancePerPulse;
	}

	@Override
	public double getRate() {
		return talon.getSelectedSensorVelocity(0) * distancePerPulse * 10;
	}

	@Override
	public double getPulses() {
		return talon.getSelectedSensorPosition(0);
	}

	@Override
	public void setDistancePerPulse(double distancePerPulse) {
		this.distancePerPulse = distancePerPulse;
	}

	public void setPhase(boolean sensorPhase) {
		talon.setSensorPhase(sensorPhase);
	}

	@Override
	public void zero() {
		talon.setSelectedSensorPosition(0, 0, Robot.CONFIG_TIMEOUT);
	}

}
