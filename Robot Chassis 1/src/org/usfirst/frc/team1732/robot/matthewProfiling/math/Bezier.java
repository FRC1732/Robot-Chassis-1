package org.usfirst.frc.team1732.robot.matthewProfiling.math;

import java.util.LinkedList;

public class Bezier {
	
	private static LinkedList<double[]> paschal = new LinkedList<>();
	static {
		paschal.add(new double[] {});
		paschal.add(new double[] {1});
		paschal.add(new double[] {1, 1});
		paschal.add(new double[] {1, 2, 1});
		paschal.add(new double[] {1, 3, 3, 1});
		paschal.add(new double[] {1, 4, 6, 4, 1});
	}
	
	public static void expand(int order) {
		while(paschal.size() < order) {
			double[] cur = new double[paschal.size()];
			double[] last = paschal.get(paschal.size()-1);
			
			cur[0] = 1;
			for(int i = 1; i < cur.length - 1; i++) {
				cur[i] = last[i-1]+last[i];
			}
			cur[cur.length - 1] = 1;
		}
	}
	
	public static double[] get(int order) {
		return paschal.get(order);
	}
	
	protected final Vector[] points;
	protected final Vector[] derivative;
	protected final Vector[] derivative2;
	protected final double[] pasc;
	
	private Vector[] getDerivative(Vector[] of) {
		Vector[] tmp = new Vector[points.length - 1];
		
		for(int i = 0; i < tmp.length; i++) {
			tmp[i] = new Vector(tmp.length * (of[i+1].getX() - of[i].getX()),
					tmp.length * (of[i+1].getY() - of[i].getY()));
		}
		
		return tmp;
	}
	
	public Bezier(Vector... points) {
		this.points = points;
		derivative = getDerivative(points);
		derivative2 = getDerivative(derivative);
		pasc = get(points.length);
	}
	
	public Vector at(double t) {
		Vector v = new Vector(0, 0);
		double tmp;
		for(int i = 0; i < points.length; i++) {
			tmp = pasc[i]*Math.pow(1-t, points.length - (i+1))*Math.pow(t, i+1);
			v.add(tmp*points[i].getX(), tmp*points[i].getY());
		}
		
		return v;
	}
	
	public Vector dAt(double t) {
		Vector v = new Vector(0, 0);
		double tmp;
		for(int i = 0; i < derivative.length; i++) {
			tmp = pasc[i]*Math.pow(1-t, derivative.length - (i+1))*Math.pow(t, i+1);
			v.add(tmp*derivative[i].getX(), tmp*derivative[i].getY());
		}
		
		return v;
	}
	
	public void axisAlign() {
		for(int i = points.length - 1; i >= 0; i--) {
			points[i].sub(points[0]);
		}
		//theta = 
		double theta = Math.atan(points[points.length - 1].getY()/points[points.length - 1].getX());
		for(Vector v: points) {
			v.rotate(theta);
		}
	}
	
	public Vector normal(double t) {
		return dAt(t).ortho();
	}
	
	public double arcLength() {
		return arcLength(100);
	}

	public double arcLength(int i) {
		double sum = 0;
		
		Vector p = this.at(0);
		Vector c;
		for(double t = 1/i; t <= 1; t+= 1/i) {
			c = this.at(t);
			sum+= p.sub(c).mag();
			p = c;
		}
		
		return sum;
	}

}
