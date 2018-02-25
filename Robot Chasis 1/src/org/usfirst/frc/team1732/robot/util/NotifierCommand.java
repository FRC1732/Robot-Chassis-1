package org.usfirst.frc.team1732.robot.util;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.hal.NotifierJNI;

// based off the Notifier class from wpilib
public abstract class NotifierCommand extends Command {

	protected abstract void init();

	@Override
	protected final void initialize() {
		init();
		startPeriodic();
	}

	protected abstract void exec();

	@Override
	protected final void execute() {
		// do nothing in here
	}

	protected abstract boolean isDone();

	@Override
	protected final boolean isFinished() {
		return !m_isRunning;
	}

	protected abstract void whenEnded();

	@Override
	protected final void end() {
		// stop(); don't need to call, stop should have been called when m_isRunning was
		// set to false
		whenEnded();
	}

	// by default calls whenEnded, but can be overridden
	protected void whenInterrupted() {
		whenEnded();
	}

	@Override
	protected final void interrupted() {
		stop();
		whenInterrupted();
	}

	// The thread waiting on the HAL alarm.
	private final Thread m_thread;

	// The lock for the process information.
	private final ReentrantLock m_processLock = new ReentrantLock();

	// The C pointer to the notifier object. We don't use it directly, it is
	// just passed to the JNI bindings.
	private final AtomicInteger m_notifier = new AtomicInteger();
	// The time, in microseconds, at which the corresponding handler should be
	// called. Has the same zero as Utility.getFPGATime().
	private double m_expirationTime = 0;
	// The handler passed in by the user which should be called at the
	// appropriate interval.
	private final Runnable m_handler;
	// The period of the calling
	private final double m_period;
	// Used to check if the notifier is running. Volatile so reads/writes are
	// directly from/to memory instead of the CPU cache
	private volatile boolean m_isRunning = false;

	// timer for time since started;
	private final Timer timer;

	@Override
	protected final void finalize() {
		int handle = m_notifier.getAndSet(0);
		NotifierJNI.stopNotifier(handle);
		// Join the thread to ensure the handler has exited.
		if (m_thread.isAlive()) {
			try {
				m_thread.interrupt();
				m_thread.join();
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
		NotifierJNI.cleanNotifier(handle);
	}

	/**
	 * Update the alarm hardware to reflect the next alarm.
	 */
	private final void updateAlarm() {
		int notifier = m_notifier.get();
		if (notifier == 0) {
			return;
		}
		NotifierJNI.updateNotifierAlarm(notifier, (long) (m_expirationTime * 1e6));
	}

	/**
	 * Create a Notifier for timer event notification.
	 */
	public NotifierCommand(int loopPeriodMilliseconds) {
		m_period = loopPeriodMilliseconds / 1000.0;
		m_handler = this::exec;
		m_notifier.set(NotifierJNI.initializeNotifier());

		m_thread = new Thread(() -> {
			while (!Thread.interrupted()) {
				int notifier = m_notifier.get();
				if (notifier == 0) {
					break;
				}
				long curTime = NotifierJNI.waitForNotifierAlarm(notifier);
				if (curTime == 0) {
					break;
				}

				Runnable handler = null;
				m_processLock.lock();
				try {
					handler = m_handler;
					m_expirationTime += m_period;
					updateAlarm();
				} finally {
					m_processLock.unlock();
				}

				if (handler != null && m_isRunning) {
					handler.run();
					// if (m_isRunning) {
					// m_handler.run();
					if (isDone()) {
						stop();
					}
				}
			}
		});
		m_thread.setDaemon(true);
		m_thread.setUncaughtExceptionHandler((thread, error) -> {
			Throwable cause = error.getCause();
			if (cause != null) {
				error = cause;
			}
			DriverStation.reportError("Unhandled exception: " + error.toString(), error.getStackTrace());
			DriverStation.reportWarning("Robots should not quit, but yours did!", false);
			DriverStation.reportError(
					"The loopFunc() method (or methods called by it) should have handled " + "the exception above.",
					false);
			System.exit(1);
		});
		m_thread.start();
		timer = new Timer();
	}

	/**
	 * Register for periodic event notification. A timer event is queued for
	 * periodic event notification. Each time the interrupt occurs, the event will
	 * be immediately requeued for the same time interval.
	 *
	 * @param period
	 *            Period in seconds to call the handler starting one period after
	 *            the call to this method.
	 */
	private final void startPeriodic() {
		m_processLock.lock();
		try {
			m_expirationTime = RobotController.getFPGATime() * 1e-6 + m_period;
			updateAlarm();
			timer.reset();
			timer.start();
			m_isRunning = true;
		} finally {
			m_processLock.unlock();
		}
	}

	/**
	 * Stop timer events from occurring. Stop any repeating timer events from
	 * occurring. This will also remove any single notification events from the
	 * queue. If a timer-based call to the registered handler is in progress, this
	 * function will block until the handler call is complete.
	 */
	private final void stop() {
		NotifierJNI.cancelNotifierAlarm(m_notifier.get());
		timer.stop();
		m_isRunning = false;
	}

	public final double timeSinceStarted() {
		return timer.get();
	}

}