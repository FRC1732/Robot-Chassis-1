package org.usfirst.frc.team1732.robot.controlutils.motionprofiling;

import java.util.Iterator;

import org.usfirst.frc.team1732.robot.Robot;

/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * 
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus		
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 * 
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 * 
 * getControlMode, to check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class DoubleProfileManager {

	private Iterator<TrajectoryPoint[]> pointIterator;

	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus, keep
	 * one copy.
	 */
	private MotionProfileStatus _leftStatus = new MotionProfileStatus();
	private MotionProfileStatus _rightStatus = new MotionProfileStatus();

	/** additional cache for holding the active trajectory point */
	double _pos = 0, _vel = 0, _heading = 0;

	/**
	 * reference to the talon we plan on manipulating. We will not changeMode() or
	 * call set(), just get motion profile status and make decisions based on motion
	 * profile.
	 */
	private TalonSRX _leftTalon;
	private TalonSRX _rightTalon;
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int _state = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a good
	 * idea to add a timeout. Set to -1 to disable. Set to nonzero to count down to
	 * '0' which will print an error message. Counting loops is not a very accurate
	 * method of tracking timeout, but this is just conservative timeout. Getting
	 * time-stamps would certainly work too, this is just simple (no need to worry
	 * about timer overflows).
	 */
	private int _loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will service
	 * it.
	 */
	private boolean _bStart = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want the
	 * set value to be and let the calling module apply it whenever we decide to
	 * switch to MP mode.
	 */
	private SetValueMotionProfile _leftSetValue = SetValueMotionProfile.Disable;
	private SetValueMotionProfile _rightSetValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion profile.
	 */
	private static final int kMinPointsInTalon = 67;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop is
	 * about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;

	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer. Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talon.processMotionProfileBuffer() in your teleop
	 * loop. Generally speaking you want to call it at least twice as fast as the
	 * duration of your trajectory points. So if they are firing every 20ms, you
	 * should call every 10ms.
	 */
	class PeriodicRunnable implements java.lang.Runnable {
		@Override
		public void run() {
			_leftTalon.processMotionProfileBuffer();
			_rightTalon.processMotionProfileBuffer();
		}
	}

	Notifier _notifer = new Notifier(new PeriodicRunnable());

	/**
	 * C'tor
	 * 
	 * @param talon
	 *            reference to Talon object to fetch motion profile status from.
	 */
	public DoubleProfileManager(TalonSRX left, TalonSRX right) {
		_leftTalon = left;
		_rightTalon = right;
		_notifer.startPeriodic(0.01);
		_leftTalon.changeMotionControlFramePeriod(10);
		_rightTalon.changeMotionControlFramePeriod(10);
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during disabled
	 * and when Talon is not in MP control mode.
	 */
	public void reset(Iterator<TrajectoryPoint[]> pointIterator) {
		this.pointIterator = pointIterator;
		/*
		 * Let's clear the buffer just in case user decided to disable in the middle of
		 * an MP, and now we have the second half of a profile just sitting in memory.
		 */
		_leftTalon.clearMotionProfileTrajectories();
		_rightTalon.clearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		_leftSetValue = SetValueMotionProfile.Disable;
		_rightSetValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		_state = 0;
		_loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next button
		 * press
		 */
		_bStart = false;
	}

	/**
	 * Called every robot loop while profile is running to keep talon in right mode
	 */
	public void run() {
		_leftTalon.set(ControlMode.MotionProfile, _leftSetValue.value);
		_rightTalon.set(ControlMode.MotionProfile, _rightSetValue.value);
		control();
	}

	/**
	 * Called every loop.
	 */
	public void control() {
		/* Get the motion profile status every loop */
		_leftTalon.getMotionProfileStatus(_leftStatus);
		_rightTalon.getMotionProfileStatus(_rightStatus);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make sure
		 * things never get stuck.
		 */
<<<<<<< HEAD
		if (_loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (_loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker tripped
				 */
				Instrumentation.OnNoProgress();
			} else {
				_loopTimeout--;
			}
		}
=======
		// if (_loopTimeout < 0) {
		// /* do nothing, timeout is disabled */
		// } else {
		// /* our timeout is nonzero */
		// if (_loopTimeout == 0) {
		// /*
		// * something is wrong. Talon is not present, unplugged, breaker tripped
		// */
		// Instrumentation.OnNoProgress();
		// } else {
		// --_loopTimeout;
		// }
		// }
>>>>>>> e99ea5bd49a0bccf07628a6f0150db7bef042af1

		/* first check if we are in MP mode */
		if (_leftTalon.getControlMode() != ControlMode.MotionProfile
				|| _rightTalon.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around using
			 * gamepads or some other mode.
			 */
			_state = 0;
			_loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp progress,
			 * and possibly interrupting MPs if thats what you want to do.
			 */
			switch (_state) {
			case 0: /* wait for application to tell us to start an MP */
				if (_bStart) {
					_bStart = false;

					_leftSetValue = SetValueMotionProfile.Disable;
					_rightSetValue = SetValueMotionProfile.Disable;
					startFilling();
					/*
					 * MP is being sent to CAN bus, wait a small amount of time
					 */
					_state = 1;
					_loopTimeout = kNumLoopsTimeout;
				}
				break;
			case 1: /*
					 * wait for MP to stream to Talon, really just the first few points
					 */
				/* do we have a minimum numberof points in Talon */
				if (_leftStatus.btmBufferCnt > kMinPointsInTalon) {
					/* start (once) the motion profile */
					_leftSetValue = SetValueMotionProfile.Enable;
					/* MP will start once the control frame gets scheduled */
					_state = 2;
					_loopTimeout = kNumLoopsTimeout;
				}
				if (_rightStatus.btmBufferCnt > kMinPointsInTalon) {
					/* start (once) the motion profile */
					_rightSetValue = SetValueMotionProfile.Enable;
					/* MP will start once the control frame gets scheduled */
					_state = 2;
					_loopTimeout = kNumLoopsTimeout;
				}
				break;
			case 2: /* check the status of the MP */
				/*
				 * if talon is reporting things are good, keep adding to our timeout. Really
				 * this is so that you can unplug your talon in the middle of an MP and react to
				 * it.
				 */
				if (_leftStatus.isUnderrun == false) {
					_loopTimeout = kNumLoopsTimeout;
				}
				if (_rightStatus.isUnderrun == false) {
					_loopTimeout = kNumLoopsTimeout;
				}
				/*
				 * If we are executing an MP and the MP finished, start loading another. We will
				 * go into hold state so robot servo's position.
				 */
				if (_leftStatus.activePointValid && _leftStatus.isLast) {
					/*
					 * because we set the last point's isLast to true, we will get here when the MP
					 * is done
					 */
					_leftSetValue = SetValueMotionProfile.Hold;
					_state = 0;
					_loopTimeout = -1;
				}
				if (_rightStatus.activePointValid && _rightStatus.isLast) {
					/*
					 * because we set the last point's isLast to true, we will get here when the MP
					 * is done
					 */
					_rightSetValue = SetValueMotionProfile.Hold;
					_state = 0;
					_loopTimeout = -1;
				}
				break;
			}

			/* Get the motion profile status every loop */
			_leftTalon.getMotionProfileStatus(_leftStatus);
			_rightTalon.getMotionProfileStatus(_rightStatus);
			_heading = _leftTalon.getActiveTrajectoryHeading();
			_pos = _leftTalon.getActiveTrajectoryPosition();
			_vel = _leftTalon.getActiveTrajectoryVelocity();

			/* printfs and/or logging */
			Instrumentation.process(_leftStatus, _pos, _vel, _heading);
		}
	}

	/**
	 * Find enum value if supported.
	 * 
	 * @param durationMs
	 * @return enum equivalent of durationMs
	 */
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

	// /** Start filling the MPs to all of the involved Talons. */
	// private void startFilling() {
	// /* since this example only has one talon, just update that one */
	// startFilling(p.Points, p.Points.length);
	// }

	private void startFilling() {

		/* did we get an underrun condition since last time we checked ? */
		if (_leftStatus.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way we never miss
			 * logging it.
			 */
			_leftTalon.clearMotionProfileHasUnderrun(0);
		}
		if (_rightStatus.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way we never miss
			 * logging it.
			 */
			_rightTalon.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer points
		 * in memory, clear it.
		 */
		_leftTalon.clearMotionProfileTrajectories();
		_rightTalon.clearMotionProfileTrajectories();

		/*
		 * set the base trajectory period to zero, use the individual trajectory period
		 * below
		 */
		_leftTalon.configMotionProfileTrajectoryPeriod(0, Robot.CONFIG_TIMEOUT);
		_rightTalon.configMotionProfileTrajectoryPeriod(0, Robot.CONFIG_TIMEOUT);

		while (pointIterator.hasNext()) {
			TrajectoryPoint[] points = pointIterator.next();
			TrajectoryPoint leftPoint = points[0];
			TrajectoryPoint rightPoint = points[1];
			leftPoint.profileSlotSelect0 = 0;
			rightPoint.profileSlotSelect1 = 0;
			leftPoint.zeroPos = false;
			rightPoint.zeroPos = false;
			_leftTalon.pushMotionProfileTrajectory(leftPoint);
			_rightTalon.pushMotionProfileTrajectory(rightPoint);

		}
	}

	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	public void startMotionProfile() {
		_bStart = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	public SetValueMotionProfile getSetLeftValue() {
		return _leftSetValue;
	}

	public SetValueMotionProfile getSetRightValue() {
		return _rightSetValue;
	}

}