package org.usfirst.frc.team1732.robot.controlutils;

public class BasicControl {

	public static double interpolate(double startOut, double startPos, double endOut, double endPos,
			double currentPos) {
		double percentDone = currentPos / (endPos - startPos);
		double additionalOut = percentDone * (endOut - startOut);
		return startOut + additionalOut;
	}
}