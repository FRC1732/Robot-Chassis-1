package org.usfirst.frc.team1732.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.command.Command;

public class TimedThread extends Thread {
	private final int nanoDelay;
	private final Runnable r;
	public TimedThread(Runnable r, int nanoDelay) {
		this.nanoDelay = nanoDelay;
		this.r = r;
	}
	
	private Supplier<Boolean> isRunning = this::isAlive;
	
	private long last = 0;
	private void sleepExactly() {
		while(System.nanoTime() - last < nanoDelay) {
			try {
				Thread.sleep(0, nanoDelay / 10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		last = System.nanoTime();
	}
	
	public void run() {
		last = System.nanoTime();
		while(!isInterrupted() && isRunning.get()) {
			r.run();
			sleepExactly();
		}
	}
	
	public void runUntil(Command c) {
		isRunning = c::isRunning;
		start();
	}
	
	public void runUntil(Supplier<Boolean> isRunning) {
		this.isRunning = isRunning;
		start();
	}
	
}
