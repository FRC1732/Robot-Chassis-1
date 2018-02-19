package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.MyIterator;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.TrajectoryPointPair;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleProfileLoader {

	public static final int MAX_POINTS_LOADED = 10000000;

	private static final int minPointsInTalon = 100;
	private static final double timeoutSec = 0.1;

	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	private volatile MyIterator pointIterator;
	private volatile Mode mode = new Mode();
	private volatile boolean printData = false;

	private MotionProfileStatus leftStatus = new MotionProfileStatus();
	private MotionProfileStatus rightStatus = new MotionProfileStatus();

	private final Timer timer = new Timer();
	private boolean timedOut = false;
	private int leftUnderruns = 0;
	private int rightUnderruns = 0;

	private int header = 0;

	public DoubleProfileLoader(TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		int period = 5;
		leftTalon.changeMotionControlFramePeriod(period);
		rightTalon.changeMotionControlFramePeriod(period);
		double periodSec = 5 / 1000.0;
		Notifier bufferLoader = new Notifier(() -> {
			leftTalon.processMotionProfileBuffer();
			rightTalon.processMotionProfileBuffer();
		});
		bufferLoader.startPeriodic(periodSec);
		Notifier controller = new Notifier(() -> {
			control();
		});
		controller.startPeriodic(periodSec);
	}

	/**
	 * Sets the new trajectory for the profile loader. Use this if you want to
	 * change trajectory mid-execution, otherwise use startProfile.
	 * 
	 * @param pointIterator
	 */
	public void setTrajectory(MyIterator pointIterator) {
		this.pointIterator = pointIterator;
		leftTalon.configMotionProfileTrajectoryPeriod(pointIterator.baseDurationMs, 0);
		rightTalon.configMotionProfileTrajectoryPeriod(pointIterator.baseDurationMs, 0);
	}

	/**
	 * Call this once to start the profile and erase currently stored trajectories
	 */
	public void startProfile(MyIterator pointIterator) {
		setTrajectory(pointIterator);
		mode = new Mode(true, STATE.WAITING, SetValueMotionProfile.Disable);
	}

	/**
	 * Call this while you want to follow the profile
	 */
	public void run() {
		Mode currentMode = mode;
		leftTalon.set(ControlMode.MotionProfile, currentMode.setValue.value);
		rightTalon.set(ControlMode.MotionProfile, currentMode.setValue.value);
	}

	// use the following method to check if we need to move on
	public boolean isWaitingAndNotStarting() {
		Mode currentMode = mode;
		return currentMode.state == STATE.WAITING && currentMode.start == false;
	}

	public boolean isHolding() {
		return mode.setValue == SetValueMotionProfile.Hold;
	}

	public boolean isTimedOut() {
		return timedOut;
	}

	public void disable() {
		mode = new Mode(false, STATE.WAITING, SetValueMotionProfile.Disable);
		run();
	}

	private void fillUntilFullOrDone() {
		MyIterator iterator = pointIterator;
		while (iterator.hasNext()
				&& !(leftTalon.getMotionProfileTopLevelBufferCount() > MAX_POINTS_LOADED
						|| rightTalon.getMotionProfileTopLevelBufferCount() > MAX_POINTS_LOADED)
				&& !(leftTalon.isMotionProfileTopLevelBufferFull() || rightTalon.isMotionProfileTopLevelBufferFull())) {
			TrajectoryPointPair pair = iterator.next();
			leftTalon.pushMotionProfileTrajectory(pair.left);
			rightTalon.pushMotionProfileTrajectory(pair.right);
		}
	}

	private void fillUntilFullOrIter(int iterations) {
		MyIterator iterator = pointIterator;
		for (int i = 0; i < iterations && iterator.hasNext() && !leftTalon.isMotionProfileTopLevelBufferFull()
				&& !rightTalon.isMotionProfileTopLevelBufferFull(); i++) {
			TrajectoryPointPair pair = iterator.next();
			leftTalon.pushMotionProfileTrajectory(pair.left);
			rightTalon.pushMotionProfileTrajectory(pair.right);
		}
	}

	private static class Mode {
		private final boolean start;
		private final STATE state;
		private final SetValueMotionProfile setValue;

		private Mode() {
			this(false, STATE.WAITING, SetValueMotionProfile.Disable);
		}

		private Mode(boolean start, STATE state, SetValueMotionProfile setValue) {
			this.start = start;
			this.state = state;
			this.setValue = setValue;
		}

	}

	private static enum STATE {
		WAITING, LOADING, EXECUTING;
	}

	private void control() {
		leftTalon.getMotionProfileStatus(leftStatus);
		rightTalon.getMotionProfileStatus(rightStatus);

		if (leftTalon.getControlMode() != ControlMode.MotionProfile
				|| rightTalon.getControlMode() != ControlMode.MotionProfile) {
			return;
			// not using robot in motion profile mode
		} else {
			Mode currentMode = mode;
			switch (currentMode.state) {
			case WAITING:
				if (currentMode.start) {
					mode = new Mode(false, STATE.LOADING, SetValueMotionProfile.Disable);
					timer.reset();
					timer.start();
					timedOut = false;
					leftUnderruns = 0;
					rightUnderruns = 0;
					header = 0;

					// clear old trajectory in bottom and top
					leftTalon.clearMotionProfileTrajectories();
					rightTalon.clearMotionProfileTrajectories();
				}
				break;
			case LOADING:
				fillUntilFullOrIter(minPointsInTalon * 2);
				if (leftStatus.btmBufferCnt >= minPointsInTalon && rightStatus.btmBufferCnt >= minPointsInTalon) {
					mode = new Mode(false, STATE.EXECUTING, SetValueMotionProfile.Enable);
				}
				break;
			case EXECUTING:
				fillUntilFullOrDone();
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
					System.out.println("Both profile executors have reached the last point. Holding.");
					mode = new Mode(false, STATE.WAITING, SetValueMotionProfile.Hold);
				}
				// other way to be done: executor tried to get a point, but bottom buffer is
				// empty (underrun), top buffer is empty, and iterator is empty. This will
				// happen if user didn't set the lastPoint flag to true for either side's last
				// point
				if (leftStatus.isUnderrun && rightStatus.isUnderrun && leftStatus.btmBufferCnt == 0
						&& rightStatus.btmBufferCnt == 0 && leftStatus.topBufferCnt == 0
						&& rightStatus.topBufferCnt == 0 && !pointIterator.hasNext()) {
					System.out.println("Profile executor has run out of points, and no more are available. Waiting.");
					mode = new Mode(false, STATE.WAITING, SetValueMotionProfile.Disable);
				}
				break;
			}

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