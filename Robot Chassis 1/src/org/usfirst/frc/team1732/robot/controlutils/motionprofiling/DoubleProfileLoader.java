package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.MyIterator;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointPair;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleProfileLoader {

	private static final int minPointsInTalon = 80;
	private static final double timeoutSec = 0.1;

	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	private volatile MyIterator<PointPair<TrajectoryPoint>> pointIterator;
	private volatile Mode mode = new Mode(false, STATE.WAITING, SetValueMotionProfile.Disable);
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
		Thread leftLoader = new Thread(() -> {
			while (!Thread.interrupted()) {
				// if (leftStatus.topBufferRem < 512) {
				leftTalon.processMotionProfileBuffer();
				// }
			}
		});
		leftLoader.setDaemon(true);
		leftLoader.start();
		Thread rightLoader = new Thread(() -> {
			while (!Thread.interrupted()) {
				// if (rightStatus.topBufferRem < 512) {
				rightTalon.processMotionProfileBuffer();
				// }
			}
		});
		rightLoader.setDaemon(true);
		rightLoader.start();
		Notifier controller = new Notifier(() -> {
			control();
		});
		controller.startPeriodic(5 / 1000.0);
	}

	/**
	 * Sets the new trajectory for the profile loader. Use this if you want to
	 * change trajectory mid-execution, otherwise use startProfile.
	 * 
	 * @param pointIterator
	 */
	public void setTrajectory(MyIterator<PointPair<TrajectoryPoint>> pointIterator) {
		this.pointIterator = pointIterator;
		leftTalon.configMotionProfileTrajectoryPeriod(pointIterator.baseDurationMs, 0);
		rightTalon.configMotionProfileTrajectoryPeriod(pointIterator.baseDurationMs, 0);
	}

	/**
	 * Call this once to start the profile and erase currently stored trajectories
	 */
	public void startProfile(MyIterator<PointPair<TrajectoryPoint>> pointIterator) {
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
		MyIterator<PointPair<TrajectoryPoint>> iterator = pointIterator;
		while (iterator.hasNext() && !leftTalon.isMotionProfileTopLevelBufferFull()
				&& !rightTalon.isMotionProfileTopLevelBufferFull()) {
			PointPair<TrajectoryPoint> pair = iterator.next();
			leftTalon.pushMotionProfileTrajectory(pair.left);
			rightTalon.pushMotionProfileTrajectory(pair.right);
		}
	}

	private void fillUntilFullOrIter(int iterations) {
		MyIterator<PointPair<TrajectoryPoint>> iterator = pointIterator;
		for (int i = 0; i < iterations && iterator.hasNext() && !leftTalon.isMotionProfileTopLevelBufferFull()
				&& !rightTalon.isMotionProfileTopLevelBufferFull(); i++) {
			PointPair<TrajectoryPoint> pair = iterator.next();
			leftTalon.pushMotionProfileTrajectory(pair.left);
			rightTalon.pushMotionProfileTrajectory(pair.right);
		}
	}

	private static class Mode {
		private final boolean start;
		private final STATE state;
		private final SetValueMotionProfile setValue;

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
				|| rightTalon.getControlMode() != ControlMode.MotionProfile
				|| DriverStation.getInstance().isDisabled()) {
			return;
			// not using robot in motion profile mode
		} else {
			Mode currentMode = mode;
			switch (currentMode.state) {
			case WAITING:
				if (currentMode.start) {
					System.out.println("MP Started");
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
					System.out.println("MP Finished Loading");
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
					printDoubleHeader();
					header = 0;
				}
				header++;
				int lP = leftTalon.getActiveTrajectoryPosition();
				int lE = leftTalon.getClosedLoopError(0);
				int lV = leftTalon.getActiveTrajectoryVelocity();
				int rP = rightTalon.getActiveTrajectoryPosition();
				int rE = rightTalon.getClosedLoopError(0);
				int rV = rightTalon.getActiveTrajectoryVelocity();
				printDoubleStatus(leftStatus, rightStatus, lP, rP, lE, rE, lV, rV);
			}
		}

	}

	public void enablePrintingData() {
		printData = true;
	}

	public void disablePrintingData() {
		printData = false;
	}

	private static void printDoubleHeader() {
		System.out.printf("%4s%4s%8s%8s%8s%6s%5s%5s%6s%5s%16s%15s%12s%9s%n", "P0", "P1", "TopCnt", "TopRem", "BtmCnt",
				"HasUn", "Mode", "IsUn", "IsVal", "Last", "Pos", "Pos Err", "Vel", "Duration");
	}

	private static void printDoubleStatus(MotionProfileStatus lS, MotionProfileStatus rS, int lP, int rP, int lE,
			int rE, int lV, int rV) {
		String formats = "%4s%4s%8s%8s%8s%6s%5s%5s%6s%5s%16s%15s%12s%9s%n";
		String profile0 = String.format("%d,%d", lS.profileSlotSelect, rS.profileSlotSelect);
		String profile1 = String.format("%d,%d", lS.profileSlotSelect1, rS.profileSlotSelect1);
		String topCount = String.format("%d,%d", lS.topBufferCnt, rS.topBufferCnt);
		String topRem = String.format("%d,%d", lS.topBufferRem, rS.topBufferRem);
		String bottomCount = String.format("%d,%d", lS.btmBufferCnt, rS.btmBufferCnt);
		String hasUnderrun = String.format("%d,%d", lS.hasUnderrun ? 1 : 0, rS.hasUnderrun ? 1 : 0);
		String mode = String.format("%d,%d", lS.outputEnable.value, rS.outputEnable.value);
		String isUnderrun = String.format("%d,%d", lS.isUnderrun ? 1 : 0, rS.isUnderrun ? 1 : 0);
		String isValid = String.format("%d,%d", lS.activePointValid ? 1 : 0, rS.activePointValid ? 1 : 0);
		String isLast = String.format("%d,%d", lS.isLast ? 1 : 0, rS.isLast ? 1 : 0);
		String pos = String.format("%d,%d", lP, rP);
		String err = String.format("%d,%d", lE, rE);
		String vel = String.format("%d,%d", lV, rV);
		String dur = String.format("%d,%d", lS.timeDurMs, rS.timeDurMs);
		System.out.printf(formats, profile0, profile1, topCount, topRem, bottomCount, hasUnderrun, mode, isUnderrun,
				isValid, isLast, pos, err, vel, dur);
	}

	// private static void printHeader() {
	// System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n",
	// "Side", "Mode", "Is Valid",
	// "Is Last", "Btm Cnt", "Top Cnt", "Top Rem", "Has Undrn", "Is Undrn",
	// "Profile0", "Profile1", "Duration",
	// "Pos", "Pos Err", "Vel");
	// }
	//
	// private static void printStatus(String name, MotionProfileStatus status,
	// double pos, double error, double vel) {
	// System.out.printf("%6s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%10s%n",
	// name,
	// status.outputEnable.name(), Boolean.toString(status.activePointValid),
	// Boolean.toString(status.isLast),
	// Integer.toString(status.btmBufferCnt), Integer.toString(status.topBufferCnt),
	// Integer.toString(status.topBufferRem), Boolean.toString(status.hasUnderrun),
	// Boolean.toString(status.isUnderrun),
	// Integer.toString(status.profileSlotSelect),
	// Integer.toString(status.profileSlotSelect1),
	// Integer.toString(status.timeDurMs), Double.toString(pos),
	// Double.toString(error), Double.toString(vel));
	// }

}