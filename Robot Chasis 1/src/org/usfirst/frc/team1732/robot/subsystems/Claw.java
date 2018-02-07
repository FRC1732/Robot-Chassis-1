package org.usfirst.frc.team1732.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends Subsystem {

	public final Spark leftSpark;
	public final Spark rightSpark;
	public final Supplier<Double> leftSparkCurrent;
	public final Supplier<Double> rightSparkCurrent;

	private final PowerDistributionPanel panel = new PowerDistributionPanel();
	public static final double rightCurrentLimit = 17;
	public static final double leftCurrentLimit = 17;

	public Claw() {
		leftSpark = MotorUtils.configSpark(0, true);
		rightSpark = MotorUtils.configSpark(1, false);
		leftSparkCurrent = () -> panel.getCurrent(4);
		rightSparkCurrent = () -> panel.getCurrent(11);
	}

	public void set(double left, double right) {
		leftSpark.set(left);
		rightSpark.set(right);
	}

	public void setIn() {
		set(-1, -1);
	}

	public void setHold() {
		set(-0, -0);
	}

	public void setOut() {
		set(1, 1);
	}

	public void setStop() {
		set(0, 0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Spark Current", leftSparkCurrent.get());
		SmartDashboard.putNumber("Right Spark Current", rightSparkCurrent.get());
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}