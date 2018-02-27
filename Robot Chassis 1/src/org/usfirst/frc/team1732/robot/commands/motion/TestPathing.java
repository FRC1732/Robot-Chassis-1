package org.usfirst.frc.team1732.robot.commands.motion;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.MyIterator;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointPair;

import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Use the drivetrain to follow a path.
 * 
 * Will erase other profiles that are currently being executed/loaded into the
 * talon.
 */
public class TestPathing extends Command {

	private final MyIterator<PointPair<TrajectoryPoint>> iterator;

	public TestPathing(MyIterator<PointPair<TrajectoryPoint>> iterator) {
		requires(Robot.drivetrain);
		this.iterator = iterator;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// make sure updated gains are applied
		Robot.drivetrain.leftMPGains.applyToTalon(Robot.drivetrain.leftTalon1, 0, 0);
		Robot.drivetrain.rightMPGains.applyToTalon(Robot.drivetrain.rightTalon1, 0, 0);
		Robot.drivetrain.profileManager.startProfile(iterator);
		Robot.drivetrain.profileManager.enablePrintingData();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drivetrain.profileManager.run();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.drivetrain.profileManager.isWaitingAndNotStarting();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.profileManager.run();
		// robot either holds last point or sits there in neutral output, depending on
		// if the trajectory's last points has the 'isLast' flag set
		Robot.drivetrain.profileManager.disablePrintingData();
		System.out.println("Path test is finished!");
	}
}
