package org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.function.Supplier;

import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.controlutils.Feedforward;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.math.BezierCurve;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.math.Curve;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.math.LineSegment;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.math.Vector;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.GenerateMotionProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionProfileConstraints;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionProfileGoal;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionProfileGoal.CompletionBehavior;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionSegment;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion.MotionState;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

/**
 * Class for making continuous-curvature paths.
 * 
 * Most basic shape combinations are continuous-curvature, I haven't tested it
 * all. There are some waypoint combinations where the paths that get made would
 * be very weird, so check all paths by graphing them
 * 
 * @author Jay
 *
 */
public final class Path {

	public final boolean driveForwards;
	private final ArrayList<PathSegment> segments = new ArrayList<>();
	private Waypoint prev;
	private MotionProfile profile;

	/**
	 * 
	 * @param first
	 *            first Waypoint
	 * @param driveForwards
	 *            if this path has the robot drive forwards or backwards
	 */
	public Path(Waypoint first, boolean driveForwards) {
		this.driveForwards = driveForwards;
		this.prev = first;
	}

	public void addWaypoints(Waypoint... waypoints) {
		for (Waypoint w : waypoints) {
			addWaypoint(w);
		}
	}

	public void addWaypoint(Waypoint w) {
		addWaypoint(w, 0.5);
	}

	public void addWaypoint(Waypoint w, double midControlPercent) {
		// System.out.println("ADDING WAYPOINT: " + w.toString());
		double theta1 = prev.heading.getAbsoluteAngle();
		double theta2 = w.heading.getAbsoluteAngle();

		Vector direction = w.position.subtract(prev.position);
		double alpha = direction.getAbsoluteAngle() - theta1;
		double beta = theta2 - theta1;

		// System.out.println("T1 " + theta1);
		// System.out.println("T2 " + theta2);
		// System.out.println("A " + alpha + ", " + Math.PI / 4);
		// System.out.println("B " + beta);
		Curve curve = null;
		if (Util.epsilonEquals(alpha, 0) && Util.epsilonEquals(beta, 0)) {
			// waypoints are colinear
			// System.out.println("Running thing 0");
			curve = new LineSegment(prev.position, w.position);
		} else if ((alpha >= 0 && beta < alpha) || (alpha <= 0 && beta > alpha)) {
			// not the same as checking if abs(beta) < abs(alpha)
			// System.out.println("Running thing 1");
			curve = addCase1(w, midControlPercent);
		} else if (Math.abs(beta) > Math.abs(alpha) && Math.abs(beta) <= 2 * Math.PI / 3) {
			// System.out.println("Running thing 2");
			curve = addCase2(w, midControlPercent);
		} else if (Math.abs(beta) > Math.abs(alpha) && Math.abs(beta) > 2 * Math.PI / 3) {
			// System.out.println("Running thing 3");
			curve = addCase3(w, midControlPercent);
		} else {
			System.err.println("INVALID WAYPOINT/BAD PATH");
		}
		if (curve != null) {
			segments.add(new PathSegment(prev, w, curve));
			prev = w;
		}
	}

	private BezierCurve addCase1(Waypoint w, double n) {
		// could have middle point be along direction at n, or be parallel at n
		// right now doing middle point along direction at n
		Vector v1 = prev.position;
		Vector v7 = w.position;
		LineSegment l17 = new LineSegment(v1, v7);
		Vector v4 = l17.getPointAtPercent(n);
		Vector h3 = prev.heading.rotate(-Math.PI / 2);
		Vector v3 = new LineSegment(v4, v4.add(h3)).getIntersection(new LineSegment(prev));
		Vector h5 = w.heading.rotate(Math.PI / 2);
		Vector v5 = new LineSegment(v4, v4.add(h5)).getIntersection(new LineSegment(w));
		Vector v2 = new LineSegment(v1, v3).getPointAtPercent(n);
		Vector v6 = new LineSegment(v5, v7).getPointAtPercent(n);
		return new BezierCurve(v1, v2, v3, v4, v5, v6, v7);
	}

	private BezierCurve addCase2(Waypoint w, double n) {
		LineSegment lineA = new LineSegment(prev);
		LineSegment lineB = new LineSegment(w);
		// System.out.println("Line A: " + lineA.toString());
		// System.out.println("Line B: " + lineB.toString());
		Vector intersection = lineA.getIntersection(lineB);
		// System.out.println(intersection);
		Vector v1 = prev.position;
		Vector v5 = w.position;
		Vector v3 = intersection;
		LineSegment ac = new LineSegment(v1, v3);
		LineSegment ce = new LineSegment(v3, v5);
		Vector v2 = ac.getPointAtPercent(n);
		Vector v4 = ce.getPointAtPercent(n);
		return new BezierCurve(v1, v2, v3, v4, v5);
	}

	private BezierCurve addCase3(Waypoint w, double n) {
		Vector v1 = prev.position;
		Vector v7 = w.position;
		LineSegment l17 = new LineSegment(v1, v7);
		Vector m4 = l17.getPointAtPercent(n);
		Vector a4 = l17.getDirection().rotate(Math.PI / 2);
		Vector v4 = m4.add(a4.scale(0.5));
		LineSegment l4 = new LineSegment(v4, v4.add(l17.getDirection()));
		Vector v3 = l4.getIntersection(new LineSegment(prev));
		Vector v5 = l4.getIntersection(new LineSegment(w));
		Vector v2 = new LineSegment(v1, v3).getPointAtPercent(n);
		Vector v6 = new LineSegment(v5, v7).getPointAtPercent(n);
		return new BezierCurve(v1, v2, v3, v4, v5, v6, v7);
	}

	public void addPathSegment(PathSegment seg) {
		segments.add(seg);
		prev = seg.end;
	}

	public void addPathSegments(PathSegment... seg) {
		segments.addAll(Arrays.asList(seg));
		prev = seg[seg.length - 1].end;
	}

	public void generateProfile(double maxVelocity, double maxAcceleration) {
		for (PathSegment seg : segments) {
			if (seg.curve instanceof BezierCurve) {
				((BezierCurve) seg.curve).makeTable();
			}
		}
		MotionProfileConstraints constraints = new MotionProfileConstraints(maxVelocity, maxAcceleration);

		MotionState previousState = new MotionState(0, 0, 0, maxAcceleration);

		// System.out.println("Total length: " +
		// segments.get(0).curve.getTotalArcLength());
		double goalPos = segments.get(0).curve.getTotalArcLength() + previousState.pos();

		MotionProfileGoal goalState = new MotionProfileGoal(goalPos, Math.abs(segments.get(0).end.vel),
				CompletionBehavior.OVERSHOOT);
		MotionProfile currentProfile = GenerateMotionProfile.generateStraightMotionProfile(constraints, goalState,
				previousState);
		previousState = currentProfile.endState();

		for (int i = 1; i < segments.size(); i++) {
			goalPos = segments.get(i).curve.getTotalArcLength() + previousState.pos();
			goalState = new MotionProfileGoal(goalPos, segments.get(i).end.vel, CompletionBehavior.OVERSHOOT);
			currentProfile.appendProfile(
					GenerateMotionProfile.generateStraightMotionProfile(constraints, goalState, previousState));
			previousState = currentProfile.endState();
		}
		profile = currentProfile;
		if (!driveForwards) {
			for (MotionSegment s : profile.segments()) {
				s.setStart(s.start().flipped());
				s.setEnd(s.end().flipped());
			}
		}
	}

	public PathSegment[] getSegments() {
		return segments.toArray(new PathSegment[segments.size()]);
	}

	public MotionProfile getProfile() {
		return profile;
	}

	public TrajectoryHolder getTrajectoryPoints(double robotWidth, double pointDurationSec) {
		int cs = 0;
		PathSegment currentSegment = segments.get(0);
		double segmentLengthSum = 0;

		double duration = profile.duration();
		int pointCount = (int) (duration / pointDurationSec);
		double increment = duration / (pointCount - 1);

		MyPoint[] left = new MyPoint[pointCount];
		MyPoint[] right = new MyPoint[pointCount];

		MotionState previousState = profile.stateByTime(0).get();
		Vector coord = currentSegment.curve.getPointAtArcLength(0);

		left[0] = new MyPoint(coord.x, coord.y, previousState.pos(), previousState.vel(), previousState.acc(),
				pointDurationSec);
		right[0] = new MyPoint(coord.x, coord.y, previousState.pos(), previousState.vel(), previousState.acc(),
				pointDurationSec);

		for (int i = 1; i < pointCount; i++) {
			MotionState state = profile.stateByTimeClamped(i * increment);
			double currentLength = currentSegment.curve.getTotalArcLength();
			if (state.pos() > currentLength + segmentLengthSum) {
				if (cs + 1 >= segments.size()) {

				} else {
					cs++;
					segmentLengthSum += currentLength;
					currentSegment = segments.get(cs);
				}
			}
			/*
			 * Because K positive is curving to the left, and K negative is to the right,
			 * the side new radii =
			 * 
			 * Math.abs(1/K-a)
			 * 
			 * for left side, and
			 * 
			 * Math.abs(1/K+a)
			 * 
			 * for right side
			 */

			double curvature = currentSegment.curve.getCurvatureAtArcLength(state.pos() - segmentLengthSum);
			double dArc = state.pos() - previousState.pos();
			coord = currentSegment.curve.getPointAtArcLength(state.pos() - segmentLengthSum);
			// System.out.println(coord);
			if (Math.abs(curvature) < 1.0E-20) {
				left[i] = new MyPoint(coord.x, coord.y, left[i - 1].position + dArc, state.vel(), state.acc(),
						pointDurationSec);
				right[i] = new MyPoint(coord.x, coord.y, right[i - 1].position + dArc, state.vel(), state.acc(),
						pointDurationSec);
			} else {
				double r = 1 / curvature;
				double lR = Math.abs(r - robotWidth / 2);
				double rR = Math.abs(r + robotWidth / 2);
				r = Math.max(lR, rR);
				double lK = lR / r;
				double rK = rR / r;
				double leftV = state.vel() * lK;
				double rightV = state.vel() * rK;
				double leftA = (leftV - left[i - 1].velocity) / pointDurationSec;
				double rightA = (rightV - right[i - 1].velocity) / pointDurationSec;
				left[i] = new MyPoint(coord.x, coord.y, left[i - 1].position + dArc * lK, leftV, leftA,
						pointDurationSec);
				right[i] = new MyPoint(coord.x, coord.y, right[i - 1].position + dArc * rK, rightV, rightA,
						pointDurationSec);
			}
			previousState = state;
		}
		return new TrajectoryHolder(left, right);
	}

	public static boolean isPossible(TrajectoryHolder points, double lKv, double lKa, double lKs, double rKv,
			double rKa, double rKs) {
		double leftMaxVolt = 0;
		double rightMaxVolt = 0;
		for (int i = 0; i < points.left.length; i++) {
			leftMaxVolt = Math.max(leftMaxVolt,
					Math.abs(points.left[i].velocity * lKv + points.left[i].acceleration * lKa + lKs));
			rightMaxVolt = Math.max(rightMaxVolt,
					Math.abs(points.right[i].velocity * rKv + points.right[i].acceleration * rKa + rKs));
		}
		return leftMaxVolt <= 12 && rightMaxVolt <= 12;
	}

	public static class TrajectoryHolder {
		public final MyPoint[] left;
		public final MyPoint[] right;

		public TrajectoryHolder(MyPoint[] left, MyPoint[] right) {
			super();
			this.left = left;
			this.right = right;
		}
	}

	/**
	 * 
	 * File format: <br>
	 * t, dt, x, y, curvature, leftPos, leftVel, leftAcc, rightPos, rightVel,
	 * rightAcc <br>
	 * 0, 0.01, 0, 0, 0, ... (more numbers in each column) <br>
	 * 
	 * @param path
	 *            written file location
	 */
	public void writeToFile(String path) {

	}

	private static double getLeftAdjust(Curve currentCurve, double robotWidth, double arcLength) {
		double curvature = currentCurve.getCurvatureAtArcLength(arcLength);
		double r = 1 / curvature;
		double lR = Math.abs(r - robotWidth / 2);
		double rR = Math.abs(r + robotWidth / 2);
		r = Math.max(lR, rR);
		return lR / r;
	}

	private static double getRightAdjust(Curve currentCurve, double robotWidth, double arcLength) {
		double curvature = currentCurve.getCurvatureAtArcLength(arcLength);
		double r = 1 / curvature;
		double lR = Math.abs(r - robotWidth / 2);
		double rR = Math.abs(r + robotWidth / 2);
		r = Math.max(lR, rR);
		return rR / r;
	}

	public Iterator<TrajectoryPoint[]> getIteratorWithOffset(TrajectoryDuration pointDuration, Feedforward leftFF,
			Feedforward rightFF, int initialLeftSensorUnits, int initialRightSensorUnits, double robotWidth,
			double sensorUnitsPerYourUnits) {
		return getIterator(pointDuration, leftFF, rightFF, initialLeftSensorUnits, initialRightSensorUnits, robotWidth,
				sensorUnitsPerYourUnits, false);
	}

	public Iterator<TrajectoryPoint[]> getIteratorZeroAtStart(TrajectoryDuration pointDuration, Feedforward leftFF,
			Feedforward rightFF, double robotWidth, double sensorUnitsPerYourUnits) {
		return getIterator(pointDuration, leftFF, rightFF, 0, 0, robotWidth, sensorUnitsPerYourUnits, true);
	}

	public Iterator<TrajectoryPoint[]> getIterator(TrajectoryDuration pointDuration, Feedforward leftFF,
			Feedforward rightFF, int initialLeftSensorUnits, int initialRightSensorUnits, double robotWidth,
			double sensorUnitsPerYourUnits, boolean zeroAtStart) {
		return new Iterator<TrajectoryPoint[]>() {
			int cs = 0;
			PathSegment currentSegment = segments.get(0);
			double segmentLengthSum = 0;

			double totalTime = profile.duration();
			double pointDurationSec = pointDuration.value / 1000.0;
			int pointCount = (int) (totalTime / (pointDurationSec));
			double increment = totalTime / (pointCount - 1);
			MotionState previousState = profile.stateByTime(0).get();

			double prevLeftPos = 0;
			double prevRightPos = 0;
			double prevLeftVel = 0;
			double prevRightVel = 0;
			int i = 0;

			@Override
			public boolean hasNext() {
				return i < pointCount;
			}

			@Override
			public TrajectoryPoint[] next() {
				TrajectoryPoint[] points = { new TrajectoryPoint(), new TrajectoryPoint() };
				TrajectoryPoint leftPoint = points[0];
				TrajectoryPoint rightPoint = points[1];
				leftPoint.timeDur = pointDuration;
				rightPoint.timeDur = pointDuration;
				leftPoint.headingDeg = 0;
				rightPoint.headingDeg = 0;
				leftPoint.isLastPoint = false;
				rightPoint.isLastPoint = false;
				leftPoint.profileSlotSelect0 = 0;
				rightPoint.profileSlotSelect1 = 0;
				leftPoint.zeroPos = false;
				rightPoint.zeroPos = false;
				if (i == 0) {
					leftPoint.zeroPos = zeroAtStart;
					rightPoint.zeroPos = zeroAtStart;
					leftPoint.position = previousState.pos() * sensorUnitsPerYourUnits + initialLeftSensorUnits;
					leftPoint.velocity = leftFF.getAppliedVoltage(previousState.vel(), previousState.acc());
					rightPoint.position = previousState.pos() * sensorUnitsPerYourUnits + initialRightSensorUnits;
					rightPoint.velocity = rightFF.getAppliedVoltage(previousState.vel(), previousState.acc());
					prevLeftPos = previousState.pos();
					prevLeftVel = previousState.vel();
					prevRightPos = previousState.pos();
					prevRightVel = previousState.vel();
				} else {
					MotionState state = profile.stateByTimeClamped(i * increment);
					double currentLength = currentSegment.curve.getTotalArcLength();
					if (state.pos() > currentLength + segmentLengthSum) {
						if (cs + 1 >= segments.size()) {

						} else {
							cs++;
							segmentLengthSum += currentLength;
							currentSegment = segments.get(cs);
						}
					}
					double curvature = currentSegment.curve.getCurvatureAtArcLength(state.pos() - segmentLengthSum);
					double dCenterArc = state.pos() - previousState.pos();

					// System.out.println(coord);
					if (Math.abs(curvature) < 1.0E-25) {
						leftPoint.position = (prevLeftPos + dCenterArc) * sensorUnitsPerYourUnits
								+ initialLeftSensorUnits;
						leftPoint.velocity = leftFF.getAppliedVoltage(state.vel(), state.acc());
						rightPoint.position = (prevRightPos + dCenterArc) * sensorUnitsPerYourUnits
								+ initialRightSensorUnits;
						rightPoint.velocity = rightFF.getAppliedVoltage(state.vel(), state.acc());
						prevLeftPos = prevLeftPos + dCenterArc;
						prevLeftVel = state.vel();
						prevRightPos = prevRightPos + dCenterArc;
						prevRightVel = state.vel();
					} else {
						double prevArcLength = previousState.pos() - segmentLengthSum;
						double arcLength = state.pos() - segmentLengthSum;
						double dArcLeft = Util.gaussQuadIntegrate64(
								(d) -> getLeftAdjust(currentSegment.curve, robotWidth, d), prevArcLength, arcLength);
						double dArcRight = Util.gaussQuadIntegrate64(
								(d) -> getRightAdjust(currentSegment.curve, robotWidth, d), prevArcLength, arcLength);

						double r = 1 / curvature;
						double lR = Math.abs(r - robotWidth / 2);
						double rR = Math.abs(r + robotWidth / 2);
						r = Math.max(lR, rR);
						double lK = lR / r;
						double rK = rR / r;
						double leftV = state.vel() * lK;
						double rightV = state.vel() * rK;
						double leftA = (leftV - prevLeftVel) / pointDurationSec;
						double rightA = (rightV - prevRightVel) / pointDurationSec;
						leftPoint.position = (prevLeftPos + dArcLeft) * sensorUnitsPerYourUnits
								+ initialLeftSensorUnits;
						leftPoint.velocity = leftFF.getAppliedVoltage(leftV, leftA);
						rightPoint.position = (prevRightPos + dArcRight) * sensorUnitsPerYourUnits
								+ initialRightSensorUnits;
						rightPoint.velocity = rightFF.getAppliedVoltage(rightV, rightA);
						prevLeftPos = prevLeftPos + dArcLeft;
						prevLeftVel = leftV;
						prevRightPos = prevRightPos + dArcRight;
						prevRightVel = rightV;
					}
					previousState = state;
				}
				if (i == pointCount - 1) {
					leftPoint.isLastPoint = true;
					rightPoint.isLastPoint = true;
				}

				i++;
				return points;
			}

		};
	}
	

	public static final double MAX_HEADING_CORRECTION = Math.PI / 60;
	public static final double PERCENT_HEADING_CORRECTION = 0.3;
	/**
	 * 
	 * @param pointDuration
	 * @param leftFF
	 * @param rightFF
	 * @param initialLeftSensorUnits
	 * @param initialRightSensorUnits
	 * @param robotWidth
	 * @param sensorUnitsPerYourUnits
	 * @param zeroAtStart
	 * @param actualHeading Navx
	 * @param theoryHeading the right side
	 * @param theoryCorrectedHeading the left side
	 * @return
	 */
	public Iterator<TrajectoryPoint[]> getIterator(TrajectoryDuration pointDuration, Feedforward leftFF,
			Feedforward rightFF, int initialLeftSensorUnits, int initialRightSensorUnits, double robotWidth,
			double sensorUnitsPerYourUnits, boolean zeroAtStart, Supplier<Double> actualHeading, Supplier<Double> theoryHeading, Supplier<Double> theoryCorrectedHeading) {
		return new Iterator<TrajectoryPoint[]>() {
			int cs = 0;
			PathSegment currentSegment = segments.get(0);
			double segmentLengthSum = 0;

			double totalTime = profile.duration();
			double pointDurationSec = pointDuration.value / 1000.0;
			int pointCount = (int) (totalTime / (pointDurationSec));
			double increment = totalTime / (pointCount - 1);
			MotionState previousState = profile.stateByTime(0).get();

			double prevLeftPos = 0;
			double prevRightPos = 0;
			double prevLeftVel = 0;
			double prevRightVel = 0;
			int i = 0;
			
			double headingCorrection = 0;

			@Override
			public boolean hasNext() {
				return i < pointCount;
			}

			@Override
			public TrajectoryPoint[] next() {
				double headingError = theoryHeading.get() - actualHeading.get();
				headingCorrection-= theoryCorrectedHeading.get();
				
				double diffrence = (headingError < MAX_HEADING_CORRECTION)? headingError : MAX_HEADING_CORRECTION;
				
				headingCorrection+= diffrence;
				
				TrajectoryPoint[] points = { new TrajectoryPoint(), new TrajectoryPoint() };
				TrajectoryPoint leftPoint = points[0];
				TrajectoryPoint rightPoint = points[1];
				
				leftPoint.timeDur = pointDuration;
				rightPoint.timeDur = pointDuration;
				leftPoint.headingDeg = 0;
				rightPoint.headingDeg = 0;
				leftPoint.isLastPoint = false;
				rightPoint.isLastPoint = false;
				leftPoint.profileSlotSelect0 = 0;
				rightPoint.profileSlotSelect1 = 0;
				leftPoint.zeroPos = false;
				rightPoint.zeroPos = false;
				if (i == 0) {
					leftPoint.zeroPos = zeroAtStart;
					rightPoint.zeroPos = zeroAtStart;
					leftPoint.position = previousState.pos() * sensorUnitsPerYourUnits + initialLeftSensorUnits;
					leftPoint.velocity = leftFF.getAppliedVoltage(previousState.vel(), previousState.acc());
					rightPoint.position = previousState.pos() * sensorUnitsPerYourUnits + initialRightSensorUnits;
					rightPoint.velocity = rightFF.getAppliedVoltage(previousState.vel(), previousState.acc());
					prevLeftPos = previousState.pos();
					prevLeftVel = previousState.vel();
					prevRightPos = previousState.pos();
					prevRightVel = previousState.vel();
					
					
				} else {
					MotionState state = profile.stateByTimeClamped(i * increment);
					double currentLength = currentSegment.curve.getTotalArcLength();
					if (state.pos() > currentLength + segmentLengthSum) {
						if (cs + 1 >= segments.size()) {

						} else {
							cs++;
							segmentLengthSum += currentLength;
							currentSegment = segments.get(cs);
						}
					}

					leftPoint.headingDeg = currentSegment.curve.getHeadingAtArcLength(state.pos() - segmentLengthSum);
					rightPoint.headingDeg = diffrence;
					
					double curvature = currentSegment.curve.getCurvatureAtArcLength(state.pos() - segmentLengthSum);
					double dCenterArc = state.pos() - previousState.pos();

					// System.out.println(coord);
					if (Math.abs(curvature) < 1.0E-25) {
						leftPoint.position = (prevLeftPos + dCenterArc) * sensorUnitsPerYourUnits
								+ initialLeftSensorUnits;
						leftPoint.velocity = leftFF.getAppliedVoltage(state.vel(), state.acc()+diffrence*PERCENT_HEADING_CORRECTION);
						rightPoint.position = (prevRightPos + dCenterArc) * sensorUnitsPerYourUnits
								+ initialRightSensorUnits;
						rightPoint.velocity = rightFF.getAppliedVoltage(state.vel(), state.acc()-diffrence*PERCENT_HEADING_CORRECTION);
						prevLeftPos = prevLeftPos + dCenterArc;
						prevLeftVel = state.vel();
						prevRightPos = prevRightPos + dCenterArc;
						prevRightVel = state.vel();
					} else {
						double prevArcLength = previousState.pos() - segmentLengthSum;
						double arcLength = state.pos() - segmentLengthSum;
						double dArcLeft = Util.gaussQuadIntegrate64(
								(d) -> getLeftAdjust(currentSegment.curve, robotWidth, d), prevArcLength, arcLength);
						double dArcRight = Util.gaussQuadIntegrate64(
								(d) -> getRightAdjust(currentSegment.curve, robotWidth, d), prevArcLength, arcLength);

						double r = 1 / curvature;
						double lR = Math.abs(r - robotWidth / 2);
						double rR = Math.abs(r + robotWidth / 2);
						r = Math.max(lR, rR);
						double lK = lR / r;
						double rK = rR / r;
						double leftV = state.vel() * lK;
						double rightV = state.vel() * rK;
						double leftA = (leftV - prevLeftVel) / pointDurationSec;
						double rightA = (rightV - prevRightVel) / pointDurationSec;
						leftPoint.position = (prevLeftPos + dArcLeft) * sensorUnitsPerYourUnits
								+ initialLeftSensorUnits;
						leftPoint.velocity = leftFF.getAppliedVoltage(leftV, leftA+diffrence*PERCENT_HEADING_CORRECTION);
						rightPoint.position = (prevRightPos + dArcRight) * sensorUnitsPerYourUnits
								+ initialRightSensorUnits;
						rightPoint.velocity = rightFF.getAppliedVoltage(rightV, rightA-diffrence*PERCENT_HEADING_CORRECTION);
						prevLeftPos = prevLeftPos + dArcLeft;
						prevLeftVel = leftV;
						prevRightPos = prevRightPos + dArcRight;
						prevRightVel = rightV;
					}
					previousState = state;
				}
				if (i == pointCount - 1) {
					leftPoint.isLastPoint = true;
					rightPoint.isLastPoint = true;
				}

				i++;
				return points;
			}

		};
	}

}