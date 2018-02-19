package org.usfirst.frc.team1732.robot.autotools;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationData {

	public static boolean closeSwitchIsLeft = false;
	public static boolean scaleIsLeft = false;
	public static boolean farSwitchIsLeft = false;
	private static String platePosition = "";

	public static boolean gotPlatePositions() {
		platePosition = DriverStation.getInstance().getGameSpecificMessage();
		if (platePosition == null || platePosition.equals("")) {
			return false;
		} else {
			closeSwitchIsLeft = platePosition.charAt(0) == 'L';
			scaleIsLeft = platePosition.charAt(1) == 'L';
			farSwitchIsLeft = platePosition.charAt(2) == 'L';
			return true;
		}
	}

	public static Alliance getAlliance() {
		return DriverStation.getInstance().getAlliance();
	}

}