package org.usfirst.frc.team1732.robot.config;

public class RobotConfig {

	public static final Node config = new Node("robot");
	static {
		int leftMasterID = 5;
		int rightMasterID = 0;
		Node drivetrain = config.addNode("drivetrain");
		drivetrain.addNode("leftTalon1").addData("CANid", leftMasterID,
				"isFollower", false, "masterCANid", leftMasterID, "isInverted", false);

		drivetrain.addNode("leftTalon2").addData("CANid", 2,
				"isFollower", true, "masterCANid", leftMasterID, "isInverted", false);

		drivetrain.addNode("leftTalon3").addData("CANid", 7,
				"isFollower", true, "masterCANid", leftMasterID, "isInverted", false);

		drivetrain.addNode("rightTalon1").addData("CANid", rightMasterID,
				"isFollower", false, "masterCANid", rightMasterID, "isInverted", true);

		drivetrain.addNode("rightTalon2").addData("CANid", 9,
				"isFollower", true, "masterCANid", rightMasterID, "isInverted", true);

		drivetrain.addNode("rightTalon3").addData("CANid", 3,
				"isFollower", true, "masterCANid", rightMasterID, "isInverted", true);

		Node otherMotors = config.addNode("otherMotors");
		otherMotors.addNode("otherTalon1").addData("CANid", 4,
				"isFollower", false, "masterCANid", 4, "isInverted", false);

		Node arm = config.addNode("arm");
		arm.addNode("talon").addData("CANid", 2,
				"isFollower", false, "masterCANid", 2, "isInverted", false);

		Node claw = config.addNode("claw");
		claw.addNode("leftSpark").addData("PWMchannel", 2, "isInverted", false, "PDPchannel", 3);
		claw.addNode("rightSpark").addData("PWMchannel", 3, "isInverted", false, "PDPchannel", 4);

		Node joysticks = config.addNode("joysticks");
		joysticks.addNode("left").addData("port", 0);
		joysticks.addNode("right").addData("port", 1);
	}

}