package org.usfirst.frc.team1732.robot.matthewProfiling.math;

public class Path {
	
	private Bezier[] beziers;
	
	public Path(Bezier... beziers) {
		this.setBeziers(beziers);
	}

	public Bezier[] getBeziers() {
		return beziers;
	}

	public void setBeziers(Bezier[] beziers) {
		this.beziers = beziers;
	}
	
	

}
