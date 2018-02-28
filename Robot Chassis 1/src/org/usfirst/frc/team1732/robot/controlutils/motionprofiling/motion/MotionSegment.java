package org.usfirst.frc.team1732.robot.controlutils.motionprofiling.motion;

/**
 * A MotionSegment is a movement from a start MotionState to an end MotionState
 * with a constant acceleration.
 */
public class MotionSegment {
	protected MotionState mStart;
	protected MotionState mEnd;

	public MotionSegment(MotionState start, MotionState end) {
		mStart = start;
		mEnd = end;
	}

	/**
	 * Verifies that:
	 * 
	 * 1. All segments have a constant acceleration.
	 * 
	 * 2. All segments have monotonic position (sign of velocity doesn't change).
	 * 
	 * 3. The time, position, velocity, and acceleration of the profile are
	 * consistent.
	 */
	public boolean isValid() {
		if (!MotionUtil.epsilonEquals(start().acc(), end().acc(), MotionUtil.kEpsilon)) {
			// Acceleration is not constant within the segment.
			System.err.println(
					"Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc());
			return false;
		}
		if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0
				&& !MotionUtil.epsilonEquals(start().vel(), 0.0, MotionUtil.kEpsilon)
				&& !MotionUtil.epsilonEquals(end().vel(), 0.0, MotionUtil.kEpsilon)) {
			// Velocity direction reverses within the segment.
			System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
			return false;
		}
		if (!start().extrapolate(end().t()).equals(end())) {
			// A single segment is not consistent.
			if (start().t() == end().t() && Double.isInfinite(start().acc())) {
				// One allowed exception: If acc is infinite and dt is zero.
				return true;
			}
			System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
			return false;
		}
		return true;
	}

	public boolean containsTime(double t) {
		return t >= start().t() && t <= end().t();
	}

	public boolean containsPos(double pos) {
		return pos >= start().pos() && pos <= end().pos() || pos <= start().pos() && pos >= end().pos();
	}

	public MotionState start() {
		return mStart;
	}

	public void setStart(MotionState start) {
		mStart = start;
	}

	public MotionState end() {
		return mEnd;
	}

	public void setEnd(MotionState end) {
		mEnd = end;
	}

	public double getVelocityAtTime(double t) {
		// double lT = mStart.t;
		// double lV = mStart.vel;
		// double rT = mEnd.t;
		// double rV = mEnd.vel;
		// return Util.cosineInterpolate(startOut, startPos, endOut, endPos, currentPos)
		return 0;
	}

	@Override
	public String toString() {
		return "Start: " + start() + ", End: " + end();
	}

}