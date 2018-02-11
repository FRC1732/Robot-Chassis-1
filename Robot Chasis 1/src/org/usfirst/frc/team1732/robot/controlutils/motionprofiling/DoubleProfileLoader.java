package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import java.util.Iterator;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleProfileLoader extends Subsystem {

	private static final int minPointsInTalon = 40;
	private static final double timeoutSec = 0.1;

	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	private Iterator<TrajectoryPoint[]> pointIterator;

	private MotionProfileStatus leftStatus = new MotionProfileStatus();
	private MotionProfileStatus rightStatus = new MotionProfileStatus();

	private STATE currentState = STATE.WAITING;
	private SetValueMotionProfile leftSetValue = SetValueMotionProfile.Disable;
	private SetValueMotionProfile rightSetValue = SetValueMotionProfile.Disable;

	private final Timer timer;
	private boolean timedOut = false;
	private int leftUnderruns = 0;
	private int rightUnderruns = 0;

	public DoubleProfileLoader(TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		this.timer = new Timer();
		int period = 5;
		Notifier notifier = new Notifier(() -> {
			leftTalon.processMotionProfileBuffer();
			rightTalon.processMotionProfileBuffer();
		});
		notifier.startPeriodic(period / 1000.0);
		leftTalon.changeMotionControlFramePeriod(period);
		rightTalon.changeMotionControlFramePeriod(period);
		leftSetValue = SetValueMotionProfile.Disable;
		rightSetValue = SetValueMotionProfile.Disable;
		currentState = STATE.WAITING;
	}

	/**
	 * Sets the new trajectory for the profile loader. If the lower talon doesn't
	 * have enough points loaded, then this will set the current state to WAITING so
	 * that the user must restart the loader
	 * 
	 * @param pointIterator
	 */
	public void setTrajectory(Iterator<TrajectoryPoint[]> pointIterator) {
		this.pointIterator = pointIterator;
	}

	/**
	 * Call this once to start the profile
	 */
	public void startProfile() {
		start = true;
	}

	/**
	 * Call this while you want to follow the profile
	 */
	public void run() {
		leftTalon.set(ControlMode.MotionProfile, leftSetValue.value);
		rightTalon.set(ControlMode.MotionProfile, rightSetValue.value);
	}

	// use the following method to check if we need to move on
	public boolean isWaiting() {
		return currentState == STATE.WAITING;
	}

	public boolean isHolding() {
		return leftSetValue.equals(SetValueMotionProfile.Hold) && rightSetValue.equals(SetValueMotionProfile.Hold);
	}

	public boolean isTimedOut() {
		return timedOut;
	}

	public void disable() {
		leftSetValue = SetValueMotionProfile.Disable;
		rightSetValue = SetValueMotionProfile.Disable;
		currentState = STATE.WAITING;
		run();
	}

	private void fillUntilFullOrDone() {
		while (pointIterator.hasNext() && !leftTalon.isMotionProfileTopLevelBufferFull()
				&& !rightTalon.isMotionProfileTopLevelBufferFull()) {
			TrajectoryPoint[] points = pointIterator.next();
			TrajectoryPoint leftPoint = points[0];
			TrajectoryPoint rightPoint = points[1];
			leftTalon.pushMotionProfileTrajectory(leftPoint);
			rightTalon.pushMotionProfileTrajectory(rightPoint);
		}
	}

	public static TrajectoryDuration getTrajectoryDuration(int durationMs) {
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError(
					"Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);
		}
		/* pass to caller */
		return retval;
	}

	@Override
	protected void initDefaultCommand() {
	}

	private static enum STATE {
		WAITING, LOADING, EXECUTING;
	}

	private boolean start;

	@Override
	public void periodic() {
		leftTalon.getMotionProfileStatus(leftStatus);
		rightTalon.getMotionProfileStatus(rightStatus);

		if (leftTalon.getControlMode() != ControlMode.MotionProfile
				|| rightTalon.getControlMode() != ControlMode.MotionProfile) {
			return;
			// not using robot in motion profile mode
		} else {
			switch (currentState) {
			case WAITING:
				if (start) {
					start = false;

					timer.reset();
					timer.start();
					timedOut = false;
					leftUnderruns = 0;
					rightUnderruns = 0;
					header = 0;

					leftSetValue = SetValueMotionProfile.Disable;
					rightSetValue = SetValueMotionProfile.Disable;
					// clear old trajectory in bottom and top
					leftTalon.clearMotionProfileTrajectories();
					rightTalon.clearMotionProfileTrajectories();
					// we will use the period from the TrajectoryPoint
					leftTalon.configMotionProfileTrajectoryPeriod(0, 0);
					rightTalon.configMotionProfileTrajectoryPeriod(0, 0);

					currentState = STATE.LOADING;
				}
				break;
			case LOADING:
				if (leftStatus.btmBufferCnt >= minPointsInTalon && rightStatus.btmBufferCnt >= minPointsInTalon) {
					leftSetValue = SetValueMotionProfile.Enable;
					rightSetValue = SetValueMotionProfile.Enable;

					currentState = STATE.EXECUTING;
				}
				break;
			case EXECUTING:
				if (leftStatus.hasUnderrun) {
					leftUnderruns++;
					leftTalon.clearMotionProfileHasUnderrun(0);
				}
				if (rightStatus.hasUnderrun) {
					rightUnderruns++;
					rightTalon.clearMotionProfileHasUnderrun(0);
				}

				if (leftStatus.isUnderrun == false && rightStatus.isUnderrun == false) {
					timer.reset();
					timedOut = false;
				}
				if (timer.get() > timeoutSec) {
					System.out.println("Have underrun for longer than " + timeoutSec
							+ " seconds. Probably not feeding points fast enough");
					timedOut = true;
				}
				// one way to be done: reach last point if user set it for both
				if (leftStatus.activePointValid && leftStatus.isLast && rightStatus.activePointValid
						&& rightStatus.isLast) {
					leftSetValue = SetValueMotionProfile.Hold;
					rightSetValue = SetValueMotionProfile.Hold;
					System.out.println("Both profile executors have reached the last point");
					currentState = STATE.WAITING;
				}
				// other way to be done: bottom, top buffer, and iterator is empty and user
				// didn't set lastpoint
				if (leftStatus.isUnderrun && rightStatus.hasUnderrun && leftStatus.topBufferCnt == 0
						&& rightStatus.topBufferCnt == 0 && !pointIterator.hasNext()) {
					System.out.println("Profile manager has run out of points");
					leftSetValue = SetValueMotionProfile.Disable;
					rightSetValue = SetValueMotionProfile.Disable;
					currentState = STATE.WAITING;
				}
				break;
			}

			fillUntilFullOrDone(); // want to start motion profiling

			leftTalon.getMotionProfileStatus(leftStatus);
			rightTalon.getMotionProfileStatus(rightStatus);

			SmartDashboard.putNumber("Left Underruns", leftUnderruns);
			SmartDashboard.putNumber("Right Underruns", rightUnderruns);
			SmartDashboard.putBoolean("Double Loader Timed Out", isTimedOut());

			if (header % 8 == 0) {
				printHeader();
				header = 0;
			}
			header++;
			printStatus("Left", leftStatus, leftTalon.getActiveTrajectoryPosition(), leftTalon.getClosedLoopError(0),
					leftTalon.getActiveTrajectoryVelocity(), leftTalon.getActiveTrajectoryHeading(),
					leftTalon.getClosedLoopError(1));
			printStatus("Right", rightStatus, rightTalon.getActiveTrajectoryPosition(),
					rightTalon.getClosedLoopError(0), rightTalon.getActiveTrajectoryVelocity(),
					rightTalon.getActiveTrajectoryHeading(), rightTalon.getClosedLoopError(1));
		}
	}

	private int header = 0;

	private static void printHeader() {
		System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n", "Side", "Is Valid",
				"Btm Cnt", "Has Undrn", "Is Undrn", "Mode", "Profile0", "Profile1", "Duration", "Top Cnt", "Top Rem",
				"Pos", "Pos Err", "Vel", "Heading", "Head Err");
	}

	private static void printStatus(String name, MotionProfileStatus status, double pos, double error, double vel,
			double heading, double err) {
		System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n", name,
				Boolean.toString(status.activePointValid), Integer.toString(status.btmBufferCnt),
				Boolean.toString(status.hasUnderrun), Boolean.toString(status.isLast),
				Boolean.toString(status.isUnderrun), status.outputEnable.name(),
				Integer.toString(status.profileSlotSelect), Integer.toString(status.profileSlotSelect1),
				Integer.toString(status.timeDurMs), Integer.toString(status.topBufferCnt),
				Integer.toString(status.topBufferRem));
	}

}