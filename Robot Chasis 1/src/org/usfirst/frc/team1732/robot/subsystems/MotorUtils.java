package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.config.Node;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;

public class MotorUtils {

	public static TalonSRX configureTalon(Node talonNode, double percentDeadband,
			int configTimout) {
		int CANid = talonNode.getData("CANid");
		TalonSRX talon = new TalonSRX(CANid);
		talon.setNeutralMode(NeutralMode.Coast);
		talon.setInverted(talonNode.getData("isInverted"));

		// we need to figure out exactly what the follower motors will "follow" (do we
		// need to configure current limit for followers too, or just master?

		boolean isFollower = talonNode.getData("isFollower");
		if (isFollower) {
			talon.set(ControlMode.Follower, talonNode.getData("masterCANid"));
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

	public static Spark configureSpark(Node sparkNode) {
		Spark spark = new Spark(sparkNode.getData("PWMchannel"));
		spark.setInverted(sparkNode.getData("isInverted"));
		return spark;
	}
}
