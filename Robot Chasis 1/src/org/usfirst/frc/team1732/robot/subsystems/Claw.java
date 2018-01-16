package org.usfirst.frc.team1732.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team1732.robot.config.Node;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Claw extends Subsystem {

	public final Spark leftSpark;
	public final Spark rightSpark;
	public final DoubleSupplier leftSparkCurrent;
	public final DoubleSupplier rightSparkCurrent;
	private final PowerDistributionPanel panel = new PowerDistributionPanel();
	public static final double currentLimit = 10;

	public Claw(Node clawNode) {
		leftSpark = MotorUtils.configureSpark(clawNode.getNode("leftSpark"));
		rightSpark = MotorUtils.configureSpark(clawNode.getNode("rightSpark"));
		leftSparkCurrent = () -> panel.getCurrent(clawNode.getNode("leftSpark").getData("PDPchannel"));
		rightSparkCurrent = () -> panel.getCurrent(clawNode.getNode("rightSpark").getData("PDPchannel"));
	}

	public void set(double left, double right) {
		leftSpark.set(left);
		rightSpark.set(right);
	}

	public void setIn() {
		set(-0.5, -0.5);
	}

	public void setOut() {
		set(0.5, 0.5);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}