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
	}

}
