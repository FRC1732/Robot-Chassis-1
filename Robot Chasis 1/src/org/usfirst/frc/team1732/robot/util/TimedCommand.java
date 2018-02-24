package org.usfirst.frc.team1732.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public abstract class TimedCommand extends Command {
	private int microDelay = 20;
	
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
		while(RobotController.getFPGATime() - last < microDelay) {
			try {
				Thread.sleep(0, microDelay * 10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		last = RobotController.getFPGATime();
	}
	
	private void loop() {
		last = RobotController.getFPGATime();
		while(isRunning() && !isFinished()) {
			exec();
			sleepExactly();
		}
	}
	
	protected abstract void exec();
	protected abstract void init();
	
}
