package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.config.Node;
import org.usfirst.frc.team1732.robot.sensors.TalonEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

	public final TalonSRX talon;
	public final TalonEncoder encoder;
	public static final double DEGREES_PER_PULSE = 10;

	public static enum ArmPositions {
		SCORING(0), UPRIGHT(0), HORIZONTAL(0), INTAKE(0);

		public final double position;

		private ArmPositions(double d) {
			position = d;
		}
	}

	public Arm(Node armNode) {
		talon = MotorUtils.configureTalon(armNode.getNode("talon"), 0, Robot.CONFIG_TIMEOUT);
		encoder = new TalonEncoder(talon);
		encoder.setDistancePerPulse(DEGREES_PER_PULSE);
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
	}

}
