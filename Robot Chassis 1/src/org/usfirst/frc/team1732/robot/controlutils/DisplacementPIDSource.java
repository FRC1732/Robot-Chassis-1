package org.usfirst.frc.team1732.robot.controlutils;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public abstract class DisplacementPIDSource implements PIDSource {
	public void setPIDSourceType(PIDSourceType pidSource) {
	}

	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

}
