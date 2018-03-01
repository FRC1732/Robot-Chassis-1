package org.usfirst.frc.team1732.robot.commands.drive;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import org.usfirst.frc.team1732.robot.commands.TurnLEDOff;
import org.usfirst.frc.team1732.robot.commands.Wait;
import org.usfirst.frc.team1732.robot.commands.autotest.TurnAngle;
import org.usfirst.frc.team1732.robot.commands.drive.TurnToCube.TurnDirection;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TurnAndDriveToCube extends CommandGroup {
	EncoderReader l = drivetrain.makeLeftEncoderReader(), r = drivetrain.makeRightEncoderReader();
	GyroReader a = sensors.navX.makeReader();

	public TurnAndDriveToCube(TurnDirection turnDir, double turnSpeed) {
		a.zero();
		addSequential(new TurnLEDOff());
		addSequential(new TurnToCube(turnDir, turnSpeed));
		addSequential(new DriveDistance(() -> -sensors.limelight.getDistanceToTarget() + 5, l, r));
		addSequential(new Wait(0.5));
		addSequential(new DriveDistance(l, r));
		addSequential(new TurnAngle(() -> -a.getAngle()));
	}
}
