package org.usfirst.frc.team1732.robot.config;

public class RobotConfig {

	public static final Node config = new Node("robot");
	static {
		Node drivetrain = config.addNode("drivetrain");

		Node rightMasterCANid = new Node("masterCANid", 5);
		drivetrain.addNode("rightTalon1").addData("CANid", rightMasterCANid.getOwnData(),
				"isFollower", false, "isInverted", false).addData(rightMasterCANid);

		drivetrain.addNode("rightTalon2").addData("CANid", 2,
				"isFollower", false, "isInverted", false).addData(rightMasterCANid);

		drivetrain.addNode("rightTalon3").addData("CANid", 7,
				"isFollower", false, "isInverted", false).addData(rightMasterCANid);

		Node leftMasterCANid = new Node("masterCANid", 0);
		drivetrain.addNode("leftTalon1").addData("CANid", leftMasterCANid.getOwnData(),
				"isFollower", true, "isInverted", false).addData(leftMasterCANid);

		drivetrain.addNode("leftTalon2").addData("CANid", 9,
				"isFollower", true, "isInverted", false).addData(leftMasterCANid);

		drivetrain.addNode("leftTalon3").addData("CANid", 3,
				"isFollower", true, "isInverted", false).addData(leftMasterCANid);

		Node otherMotors = config.addNode("otherMotors");
		otherMotors.addNode("otherTalon1").addData("CANid", 4,
				"isFollower", false, "masterCANid", 4, "isInverted", false);

		Node arm = config.addNode("arm");
		arm.addNode("talon").addData("CANid", 2,
				"isFollower", false, "masterCANid", 2, "isInverted", true);

		Node claw = config.addNode("claw");
		claw.addNode("leftSpark").addData("PWMchannel", 0, "isInverted", true, "PDPchannel", 4);
		claw.addNode("rightSpark").addData("PWMchannel", 1, "isInverted", false, "PDPchannel", 11);

		Node joysticks = config.addNode("joysticks");
		joysticks.addNode("left").addData("port", 0);
		joysticks.addNode("right").addData("port", 1);
		joysticks.addNode("buttons").addData("port", 2);
	}

}