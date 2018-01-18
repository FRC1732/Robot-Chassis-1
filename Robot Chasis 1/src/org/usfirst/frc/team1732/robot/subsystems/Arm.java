package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.config.Node;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderBase;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

	public final TalonSRX talon;
	public final EncoderBase talonEncoder;
	public final EncoderReader encoder;

	public static final double DEGREES_PER_PULSE = 180.0 / 7324;

	public static enum ArmPositions {
		SCORE(200), UPRIGHT(160), HORIZONTAL(73), INTAKE(0);

		public final double position;

		private ArmPositions(double d) {
			position = d;
		}
	}

	public Arm(Node armNode) {
		talon = MotorUtils.configureTalon(armNode.getNode("talon"), 0, Robot.CONFIG_TIMEOUT);
		talonEncoder = new TalonEncoder(talon);
		talonEncoder.setDistancePerPulse(DEGREES_PER_PULSE);
		encoder = new EncoderReader(talonEncoder);
		encoder.zero();
		SmartDashboard.putNumber("LeverArmToVoltage", leverArmToVoltage);
	}

	public void setUp() {
		setSpeed(0.5);
	}

	public void setDown() {
		setSpeed(-0.5);
	}

	public void setStop() {
		setSpeed(0.0);
	}

	public void setSpeed(double speed) {
		talon.set(ControlMode.PercentOutput, speed);
	}

	@Override
	protected void initDefaultCommand() {

	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
		SmartDashboard.putNumber("Arm Pulses", talonEncoder.getPulses());
		SmartDashboard.putNumber("Arm Voltage %", talon.getMotorOutputPercent());
		leverArmToVoltage = SmartDashboard.getNumber("LeverArmToVoltage", leverArmToVoltage);
	}

	private static final double restingOffset = 90 - 65;
	private static final double armLength = 25.5;
	private double leverArmToVoltage = 0.01;

	public double getTorqueCompensation(double angle) {
		angle = angle + restingOffset;
		double leverArm;
		if (angle < 90) {
			angle = 90 - angle;
			leverArm = Math.cos(Math.toRadians(angle)) * armLength;
		}
		if (90 < angle && angle < 180) {
			leverArm = Math.cos(Math.toRadians(angle)) * armLength;
			leverArm = -leverArm;
		} else {
			leverArm = Math.cos(Math.toRadians(angle)) * armLength;
		}
		return leverArm * leverArmToVoltage;
	}

}
