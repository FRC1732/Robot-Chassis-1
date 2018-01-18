package org.usfirst.frc.team1732.robot.autotools;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationData {

	public static TeeterTotter closeSwitch;
	public static TeeterTotter scale;
	public static TeeterTotter farSwitch;

	private static String platePosition = "";

	public static void pollPlatePosition() {
		platePosition = DriverStation.getInstance().getGameSpecificMessage();
		if (platePosition.length() >= 3) {
			Alliance a = getAlliance();
			closeSwitch = new TeeterTotter(a, platePosition.charAt(0));
			scale = new TeeterTotter(a, platePosition.charAt(1));
			farSwitch = new TeeterTotter(a, platePosition.charAt(2));
		}
	}

	public static Alliance getAlliance() {
		return DriverStation.getInstance().getAlliance();
	}

	public static class TeeterTotter {

		public final Alliance leftPlateAlliance;
		public final Alliance rightPlateAlliance;

		public TeeterTotter(Alliance ourAlliance, char side) {
			if (side == 'L') {
				leftPlateAlliance = ourAlliance;
				rightPlateAlliance = getOpposite(ourAlliance);
			} else {
				leftPlateAlliance = getOpposite(ourAlliance);
				rightPlateAlliance = ourAlliance;
			}
		}

		private static Alliance getOpposite(Alliance a) {
			return a.equals(Alliance.Red) ? Alliance.Blue : Alliance.Red;
		}
	}
}