package org.usfirst.frc.team1732.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public abstract class ThreadCommand extends Command {
	private int microDelay = 20 * 1000; // 20 milliseconds

	public int getMicroDelay() {
		return microDelay;
	}

	public void setMicroDelay(int microDelay) {
		this.microDelay = microDelay;
	}

	public void setDelay(int msDelay) {
		this.microDelay = msDelay * 1000;
	}

	@Override
	protected void initialize() {
		init();
		new Thread(this::loop).start();
	}

	private long last = 0;

	private void sleepExactly() {
		while (RobotController.getFPGATime() - last < microDelay && !Thread.interrupted()) {
			try {
				Thread.sleep(0, microDelay / 100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		last = RobotController.getFPGATime();
	}

	// Timer test = new Timer();

	private void loop() {
		last = RobotController.getFPGATime();
		// test.reset();
		// test.start();
		while (isRunning() && !isFinished()) {
			exec();
			sleepExactly();
			// System.out.println("Loop time: " + test.get());
			// test.reset();
			// test.start();
		}
	}

	protected abstract void exec();

	protected abstract void init();

}
