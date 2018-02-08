package org.usfirst.frc.team1732.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;

public class MotorUtils {

	public static TalonSRX configTalon(int ownID, boolean isInverted, TalonConfiguration config) {
		TalonSRX talon = new TalonSRX(ownID);
		talon.setInverted(isInverted);

		int configTimeout = TalonConfiguration.CONFIG_TIMEOUT;
		talon.setNeutralMode(config.neutralMode);
		printError(talon.configNeutralDeadband(config.neutralDeadbandPercent, configTimeout));
		printError(talon.configNominalOutputForward(config.nominalForward, configTimeout));
		printError(talon.configNominalOutputReverse(config.nominalReverse, configTimeout));
		printError(talon.configPeakOutputForward(config.peakOutputForward, configTimeout));
		printError(talon.configPeakOutputReverse(config.peakOutputReverse, configTimeout));

		printError(talon.configOpenloopRamp(config.openLoopRamp, configTimeout));

		talon.enableVoltageCompensation(config.enableVoltageCompensation);
		printError(talon.configVoltageCompSaturation(config.voltageCompensationSaturation, configTimeout));

		talon.enableCurrentLimit(false);
		printError(talon.configContinuousCurrentLimit(config.continousCurrentLimit, configTimeout));
		printError(talon.configPeakCurrentDuration(config.peakCurrentDuration, configTimeout));
		printError(talon.configPeakCurrentLimit(config.peakCurrentLimit, configTimeout));

		printError(talon.configVelocityMeasurementPeriod(config.velocityMeasurementPeriod, configTimeout));
		printError(talon.configVelocityMeasurementWindow(config.velocityMeasurementWindow, configTimeout));

		printError(talon.configVoltageMeasurementFilter(config.voltageMeasurementWindow, configTimeout));

		printError(talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.generalStatusPeriod, configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.generalStatusPeriod,
				configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.quadratureStatusPeriod,
				configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
				config.analogTemperatureBatteryStatusPeriod, configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
				config.currentMPtargetStatusPeriod, configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, config.PIDerrorStatusPeriod,
				configTimeout));
		printError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.controlFramePeriod,
				configTimeout));

		return talon;
	}

	private static void printError(ErrorCode e) {
		if (!e.equals(ErrorCode.OK)) {
			System.err.println("Error, see docs: " + e.value + "= " + e.name());
		}
	}

	public static TalonSRX configFollowerTalon(TalonSRX follower, TalonSRX master) {
		follower.follow(master);
		return follower;
	}

	public static Spark configSpark(int pwmChannel, boolean isInverted) {
		Spark spark = new Spark(pwmChannel);
		spark.setInverted(isInverted);
		return spark;
	}
}
