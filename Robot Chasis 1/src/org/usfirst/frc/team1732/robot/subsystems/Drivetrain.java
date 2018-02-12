package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.controlutils.ClosedLoopProfile;
import org.usfirst.frc.team1732.robot.controlutils.Feedforward;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.DoubleProfileLoader;
import org.usfirst.frc.team1732.robot.drivercontrol.DifferentialDrive;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	public final TalonSRX leftTalon1;
	public final TalonSRX rightTalon1;

	public DifferentialDrive drive;

	public final TalonEncoder leftEncoder;
	public final TalonEncoder rightEncoder;

	public static final double INPUT_DEADBAND = 0.025; // 2.5%.
	public static final double MIN_OUTPUT = 0.0;
	public static final double MAX_OUTPUT = 1.0;
	public static final double ENCODER_INCHES_PER_PULSE = 0.002099;

	// Forward feedforward
	public final Feedforward leftFFF = new Feedforward(0.061137, 0.010011, 1.688535);
	public final Feedforward rightFFF = new Feedforward(0.0597575, 0.0099831, 1.7542480);
	// Backward feedforward
	public final Feedforward leftBFF = new Feedforward(0, 0, 0);
	public final Feedforward rightBFF = new Feedforward(0, 0, 0);

	// keep in mind for these
	public final ClosedLoopProfile leftMPGains = new ClosedLoopProfile("Left PID", 1.28, 0, 0,
			Feedforward.TALON_SRX_FF_GAIN, 0, 0, 0, 0);
	public final ClosedLoopProfile rightMPGains = new ClosedLoopProfile("Right PID", 1.28, 0, 0,
			Feedforward.TALON_SRX_FF_GAIN, 0, 0, 0, 0);

	/*
	 * The following 2 values are determined from the feedforward constants.
	 * 
	 * Max vel is determined by doing (12 - voltIntercept)/2.0 / kV
	 * 
	 * Max acc is determined by doing (12 - voltIntercept)/2.0 / kA
	 * 
	 * This makes it possible to be accelerating at max acceleration near max
	 * velocity (because we cannot exceed a voltage draw of 12V). If we made
	 * acceleration continuous by instead changing 'jerk', then we would be able to
	 * set these slightly higher, depending on how high we set the jerk value. Using
	 * a very high jerk value would not be any different than what we have now, but
	 * using too low of a jerk value would cause the robot to accelerate slowly.
	 */
	public static final double MAX_IN_SEC = 85.021; // max vel
	public static final double MAX_IN_SEC2 = 514.082; // max acc

	public static final double ROBOT_WIDTH_IN = 29;

	public final DoubleProfileLoader profileManager;

	public Drivetrain() {
		int leftMaster = 1;
		leftTalon1 = MotorUtils.configTalon(leftMaster, false, TalonConfiguration.DEFAULT_CONFIG);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(9, false, TalonConfiguration.DEFAULT_CONFIG), leftTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(3, false, TalonConfiguration.DEFAULT_CONFIG), leftTalon1);

		int rightMaster = 5;
		rightTalon1 = MotorUtils.configTalon(rightMaster, true, TalonConfiguration.DEFAULT_CONFIG);

		MotorUtils.configFollowerTalon(MotorUtils.configTalon(6, true, TalonConfiguration.DEFAULT_CONFIG), rightTalon1);
		MotorUtils.configFollowerTalon(MotorUtils.configTalon(7, true, TalonConfiguration.DEFAULT_CONFIG), rightTalon1);

		drive = new DifferentialDrive(leftTalon1, rightTalon1, ControlMode.PercentOutput, MIN_OUTPUT, MAX_OUTPUT,
				INPUT_DEADBAND);

		ClosedLoopProfile.applyZeroGainToTalon(leftTalon1, 0, 1);
		ClosedLoopProfile.applyZeroGainToTalon(rightTalon1, 0, 1);
		leftMPGains.applyToTalon(leftTalon1, 0, 0);
		rightMPGains.applyToTalon(rightTalon1, 0, 0);

		// two of these control the shifting. I didn't want to follow the wires
		// new Solenoid(1).set(true);
		// new Solenoid(2).set(true);
		// new Solenoid(3).set(true);

		leftEncoder = new TalonEncoder(leftTalon1, FeedbackDevice.QuadEncoder);
		rightEncoder = new TalonEncoder(rightTalon1, FeedbackDevice.QuadEncoder);
		leftEncoder.setPhase(true);
		rightEncoder.setPhase(true);
		leftEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.setDistancePerPulse(ENCODER_INCHES_PER_PULSE);
		rightEncoder.zero();
		leftEncoder.zero();

		profileManager = new DoubleProfileLoader(leftTalon1, rightTalon1);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Velocity", leftEncoder.getRate());
		SmartDashboard.putNumber("Right Velocity", rightEncoder.getRate());
		SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());
		SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
		SmartDashboard.putNumber("Left Voltage", leftTalon1.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Voltage", rightTalon1.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Pulses", leftEncoder.getPulses());
		SmartDashboard.putNumber("Left Pulses", rightEncoder.getPulses());

	}

	public EncoderReader makeRightEncoderReader() {
		return rightEncoder.makeReader();
	}

	public EncoderReader makeLeftEncoderReader() {
		return leftEncoder.makeReader();
	}

	public void setStop() {
		drive.tankDrive(0, 0);
	}

}