package org.usfirst.frc.team1732.robot.motionProfiling;

import java.util.Iterator;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

public class MotionProfile implements Iterable<TrajectoryPoint>, Iterator<TrajectoryPoint>{
	private double maxSpeed = 3.0;// rotations/second
	private double dist = 5;// rotations
	private double t1 = 400;// ms
	private double t2 = 200;// ms
	private double itp = 10;// ms
	
	private double t4;
	private double fl1;
	private double fl2;
	private double n;
	
	private int pointOn = 0;
	
	/**
	 * @param max speed of the robot (rotations / second)
	 * @param distance to travel (rotations)
	 * @param time to accelerate to full (ms)
	 * @param time to decelerate to zero (ms)
	 * @param time per point (ms)
	 */
	public MotionProfile(double maxSpeed, double dist, double t1, double t2, double itp) {
		this.maxSpeed = maxSpeed;
		this.dist = dist;
		this.t1 = t1;
		this.t2 = t2;
		this.itp = itp;
		pointOn = 0;
		
		t4 = (dist / maxSpeed)*1000;
		fl1 = Math.ceil(t1/itp);
		fl2 = Math.ceil(t2/itp);
		n = t4/itp;
		
		sum1 = new double[(int) fl2];
	}

	@Override
	public boolean hasNext() {
		return false;
	}
	
	TrajectoryPoint prev = null;
	private double psum1 = 0;
	private double psum2 = 0;
	
	private double[] sum1;
	
	@Override
	public TrajectoryPoint next() {
		TrajectoryPoint t = new TrajectoryPoint();
		t.timeDur = TrajectoryDuration.Trajectory_Duration_10ms;
		
		if(prev == null) {
			t.velocity = 0;
			t.position = 0;
			t.zeroPos = true;
			t.isLastPoint = false;
			
			pointOn++;
			prev = t;
			return t;
		}
		
		double in = (pointOn > n+3)?1:0;
		double sum1 = Math.max(0, Math.min(1, psum1 + ((in==1)?1/fl1:-1/fl1)));
		this.sum1[pointOn % this.sum1.length] = sum1;
		
		double sum2 = sum(this.sum1);
		
		t.velocity =  ((sum1+sum2)/(1+fl2))*maxSpeed;
		t.position = ((((t.velocity+prev.velocity)/2)*itp)/1000)+prev.position;
		t.zeroPos = false;
		t.isLastPoint = sum1+sum2 == 0;

		pointOn++;
		prev = t;
		return t;
	}

	private static double sum(double[] array) {
		double sum = 0;
		
		for(double cur : array) {
			sum+= cur;
		}
		
		return sum;
	}

	@Override
	public Iterator<TrajectoryPoint> iterator() {
		// TODO Auto-generated method stub
		return this;
	}
	
	public void reset() {
		pointOn = 0;
	}

	public double getMaxSpeed() {
		return maxSpeed;
	}

	public double getDist() {
		return dist;
	}

	public double getT1() {
		return t1;
	}

	public double getT2() {
		return t2;
	}

	public double getItp() {
		return itp;
	}

}
