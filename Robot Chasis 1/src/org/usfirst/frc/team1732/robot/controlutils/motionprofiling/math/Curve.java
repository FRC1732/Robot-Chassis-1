package org.usfirst.frc.team1732.robot.controlutils.motionprofiling.math;

public interface Curve {

	public double getTotalArcLength();

	/**
	 * 
	 * @param arcLength
	 *            arc length along curve
	 * @return curvature. Positive curvature indicates that left side is inner side,
	 *         negative curvature indicates that right side is inner side
	 */
	public double getCurvatureAtArcLength(double arcLength);

	public Vector getPointAtArcLength(double arcLength);

	public double getHeadingAtArcLength(double arcLength);
}
