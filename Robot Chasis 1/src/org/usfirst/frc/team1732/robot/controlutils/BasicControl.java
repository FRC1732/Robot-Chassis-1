package org.usfirst.frc.team1732.robot.controlutils;

public class BasicControl {

	public static double linearInterpolate(double startOut, double startPos, double endOut, double endPos,
			double currentPos) {
		double percentDone = (currentPos - startPos) / (endPos - startPos);
		return startOut * (1 - percentDone) + endOut * percentDone;
	}

	public static double cosineInterpolate(double startOut, double startPos, double endOut, double endPos,
			double currentPos) {
		double percentDone = (currentPos - startPos) / (endPos - startPos);
		double modifiedPD = (1 - Math.cos(percentDone * Math.PI)) / 2;
		return startOut * (1 - modifiedPD) + endOut * modifiedPD;
	}
}