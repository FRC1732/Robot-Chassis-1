package org.usfirst.frc.team1732.robot.sensors.encoders;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;

public class Tracking {
	private final TalonEncoder left;
	private final TalonEncoder right;
	
	public Tracking(TalonEncoder left, TalonEncoder right) {
		this.left = left;
		this.right = right;
	}

	private double heading = 0;
	private double x = 0;
	private double y = 0;
	private double new_heading = 0;
	private double new_x = 0;
	private double new_y = 0;
	
	public void addPoint() {
		double timeDelta = Robot.getFps();
		double leftDelta = left.getRate() * timeDelta;
		double rightDelta = right.getRate() * timeDelta;
		
		if (Math.abs(leftDelta - rightDelta) < 1.0e-6) { // basically going straight
		    new_x = x + leftDelta * Math.cos(heading);
		    new_y = y + rightDelta * Math.sin(heading);
		    new_heading = heading;
		} else {
		    double R = Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta)),
		          wd = (rightDelta - leftDelta) / Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN;

		    new_x = x + R * Math.sin(wd + heading) - R * Math.sin(heading);
		    new_y = y - R * Math.cos(wd + heading) + R * Math.cos(heading);
		    new_heading = /*boundAngle*/(heading + wd);
		}
		heading = new_heading;
		x = new_x;
		y = new_y;
	}
	
	private boolean looping;
	public void loop() {
		looping = true;
		while(looping) {
			addPoint();
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
			}
		}
	}
	
	public void stop() {
		looping = false;
	}

	public double getHeading() {
		return heading;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
}
