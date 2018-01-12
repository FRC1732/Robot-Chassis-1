package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.w3c.dom.Element;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MotorUtils {

	public static TalonSRX configureTalon(Element talonElement, double percentDeadband, int configTimout) {
		int CANid = ConfigUtils.getInteger(talonElement, "CANid");
		TalonSRX talon = new TalonSRX(CANid);
		talon.setNeutralMode(NeutralMode.Coast);
		talon.setInverted(ConfigUtils.getBoolean(talonElement, "isInverted"));
		// we need to figure out exactly what the follower motors will "follow" (do we
		// need to configure current limit for followers too, or just master?

		boolean isFollower = ConfigUtils.getBoolean(talonElement, "isFollower");
		if (isFollower) {
			talon.set(ControlMode.Follower, ConfigUtils.getInteger(talonElement, "masterCANid"));
		} else {
			// I have methods commented out here that we might want to use, but am waiting
			// for more documentation

			talon.configNeutralDeadband(percentDeadband, configTimout);
			talon.configNominalOutputForward(+0, configTimout);
			talon.configNominalOutputReverse(-0, configTimout);
			talon.configPeakOutputForward(+1.0, configTimout);
			talon.configPeakOutputReverse(-1.0, configTimout);

			// talon.configOpenloopRamp(secondsFromNeutralToFull, configTimeoutMS);
			// talon.enableVoltageCompensation(false);

			// talon.enableCurrentLimit(false);
			// talon.configContinuousCurrentLimit(amps, configTimeoutMS);
			// talon.configPeakCurrentDuration(milliseconds, configTimeoutMS);
			// talon.configPeakCurrentLimit(amps, configTimeoutMS)

			/*
			 * can set very specific frame periods (how frequently CAN data is sent)
			 * 
			 * talon.setControlFramePeriod(ControlFrame.Control_3_General, 20);
			 * talon.setStatusFramePeriod(StatusFrame.Status_1_General, periodMs,
			 * configTimeoutMS)
			 * talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMs,
			 * configTimeoutMS)
			 */

		}
		return talon;
	}
}
