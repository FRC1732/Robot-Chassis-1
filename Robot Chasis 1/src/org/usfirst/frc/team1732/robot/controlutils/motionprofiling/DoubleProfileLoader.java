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

	private boolean start;

	private SetValueMotionProfile leftSetValue = SetValueMotionProfile.Disable;
	private SetValueMotionProfile rightSetValue = SetValueMotionProfile.Disable;
	private STATE currentState = STATE.WAITING;

	private MotionProfileStatus leftStatus = new MotionProfileStatus();
	private MotionProfileStatus rightStatus = new MotionProfileStatus();

	private final Timer timer = new Timer();
	private boolean timedOut = false;
	private int leftUnderruns = 0;
	private int rightUnderruns = 0;

	private boolean printData = false;
	private int header = 0;

	public DoubleProfileLoader(TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		int period = 5;
		Notifier notifier = new Notifier(() -> {
			leftTalon.processMotionProfileBuffer();
			rightTalon.processMotionProfileBuffer();
		});
		notifier.startPeriodic(period / 1000.0);
		leftTalon.changeMotionControlFramePeriod(period);
		rightTalon.changeMotionControlFramePeriod(period);
	}

	/**
	 * Sets the new trajectory for the profile loader. Use this if you want to
	 * change trajectory mid-execution, otherwise use startProfile.
	 * 
	 * @param pointIterator
	 */
	public void setTrajectory(Iterator<TrajectoryPoint[]> pointIterator) {
		this.pointIterator = pointIterator;
	}

	/**
	 * Call this once to start the profile and erase currently stored trajectories
	 */
	public void startProfile(Iterator<TrajectoryPoint[]> pointIterator) {
		start = true;
		setTrajectory(pointIterator);
		leftSetValue = SetValueMotionProfile.Disable;
		rightSetValue = SetValueMotionProfile.Disable;
		currentState = STATE.WAITING;
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
					DriverStation.reportError("Profile Loader has underrun for longer than " + timeoutSec
							+ " seconds. Probably not feeding points fast enough", false);
					timedOut = true;
				}
				// one way to be done: reach last point if user set it for both
				if (leftStatus.activePointValid && leftStatus.isLast && rightStatus.activePointValid
						&& rightStatus.isLast) {
					leftSetValue = SetValueMotionProfile.Hold;
					rightSetValue = SetValueMotionProfile.Hold;
					System.out.println("Both profile executors have reached the last point. Holding.");
					currentState = STATE.WAITING;
				}
				// other way to be done: executor tried to get a point, but bottom buffer is
				// empty (underrun), top buffer is empty, and iterator is empty. This will
				// happen if user didn't set the lastPoint flag to true for either side's last
				// point
				if (leftStatus.isUnderrun && rightStatus.isUnderrun && leftStatus.btmBufferCnt == 0
						&& rightStatus.btmBufferCnt == 0 && leftStatus.topBufferCnt == 0
						&& rightStatus.topBufferCnt == 0 && !pointIterator.hasNext()) {
					System.out.println("Profile executor has run out of points, and no more are available. Waiting.");
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

			if (printData) {
				if (header % 8 == 0) {
					printHeader();
					header = 0;
				}
				header++;
				printStatus("Left", leftStatus, leftTalon.getActiveTrajectoryPosition(),
						leftTalon.getClosedLoopError(0), leftTalon.getActiveTrajectoryVelocity());
				printStatus("Right", rightStatus, rightTalon.getActiveTrajectoryPosition(),
						rightTalon.getClosedLoopError(0), rightTalon.getActiveTrajectoryVelocity());
			}
		}
	}

	public void enablePrintingData() {
		printData = true;
	}

	public void disablePrintingData() {
		printData = false;
	}

	private static void printHeader() {
		System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n", "Side", "Mode", "Is Valid",
				"Is Last", "Btm Cnt", "Top Cnt", "Top Rem", "Has Undrn", "Is Undrn", "Profile0", "Profile1", "Duration",
				"Pos", "Pos Err", "Vel");
	}

	private static void printStatus(String name, MotionProfileStatus status, double pos, double error, double vel) {
		System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n", name,
				status.outputEnable.name(), Boolean.toString(status.activePointValid), Boolean.toString(status.isLast),
				Integer.toString(status.btmBufferCnt), Integer.toString(status.topBufferCnt),
				Integer.toString(status.topBufferRem), Boolean.toString(status.hasUnderrun),
				Boolean.toString(status.isUnderrun), Integer.toString(status.profileSlotSelect),
				Integer.toString(status.profileSlotSelect1), Integer.toString(status.timeDurMs), Double.toString(pos),
				Double.toString(error), Double.toString(vel));
	}

}