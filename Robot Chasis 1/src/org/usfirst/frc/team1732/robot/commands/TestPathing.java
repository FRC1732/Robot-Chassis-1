package org.usfirst.frc.team1732.robot.commands;

import java.util.Iterator;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.GainProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Waypoint;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestPathing extends Command {

	private Path path;

	public TestPathing() {
		requires(Robot.drivetrain);
		Timer t = new Timer();
		t.reset();
		t.start();
		path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		path.addWaypoint(new Waypoint(0, 100, Math.PI / 2, 0));
		path.generateProfile(Drivetrain.MAX_IN_SEC, Drivetrain.MAX_IN_SEC2);
		System.out.println("Time to make: " + t.get());
		path.setPathVars(Robot.drivetrain.leftFFF, Robot.drivetrain.rightFFF, 29,
				0.01, 1.0 / Drivetrain.ENCODER_INCHES_PER_PULSE);
		Robot.drivetrain.profileManager.reset(path.iterator());
		GainProfile leftGain = Robot.drivetrain.leftGains.clone();
		leftGain.setFF(Robot.drivetrain.leftFFF);
		GainProfile rightGain = Robot.drivetrain.rightGains.clone();
		rightGain.setFF(Robot.drivetrain.rightFFF);

		leftGain.applyToTalon(Robot.drivetrain.leftTalon1, 0);
		rightGain.applyToTalon(Robot.drivetrain.rightTalon1, 0);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.leftEncoder.zero();
		Robot.drivetrain.rightEncoder.zero();
		Robot.drivetrain.profileManager.startMotionProfile();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drivetrain.profileManager.run();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
//		return Robot.drivetrain.profileManager.getSetLeftValue().equals(SetValueMotionProfile.Hold)
//				&& Robot.drivetrain.profileManager.getSetRightValue().equals(SetValueMotionProfile.Hold);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Path test is finished!");
	}
}
