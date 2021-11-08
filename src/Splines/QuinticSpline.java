package Splines;

import java.util.ArrayList;

public class QuinticSpline {
	double samplenum = 100;
	public double length = 0;
	public double tAdjLength = 0;
	public ArrayList<Vector2> Pathpoints = new ArrayList<>();
	public ArrayList<Splines.Waypoint> Waypoints = new ArrayList<>();
	public ArrayList<Segment> Segments = new ArrayList<>();
	
	public ArrayList<Vector2> LengthByTime = new ArrayList<>();
	
	public QuinticSpline(ArrayList<Splines.Waypoint> DefinePath) {
		this.Waypoints = DefinePath;
	}

	public void generateSegments() {
		for(int i = 0; i < Waypoints.size() - 1; i ++) {
			Segments.add(new Segment(Waypoints.get(i).position, Waypoints.get(i + 1).position, Waypoints.get(i).velocity,
					Waypoints.get(i + 1).velocity, Waypoints.get(i).acceleration, Waypoints.get(i + 1).acceleration));
		}
	}
	public void sampleSegments(double samples) {
		samplenum = samples;
		for(int i = 0; i < Segments.size(); i ++) {
			for(double j = 0; j < 1; j += 1/samples) {
				Segments.get(i).updateArray(j);
				Pathpoints.add(new Vector2(Segments.get(i).interpolate().x, Segments.get(i).interpolate().y));
				//System.out.println(Segments.get(i).interpolate().x);
			}
		}
		Pathpoints.add(new Vector2(Waypoints.get(Waypoints.size() - 1).position.x, Waypoints.get(Waypoints.size() - 1).position.y));
	}
	public void sampleSegments() {
		sampleSegments(samplenum);
	}
	public void integrateLength(double scaleFactor) {
		double currentSum = 0;
		double currentThetaAdjustedSum = 0;
		for(int i = Pathpoints.size() - 1; i > 0; i--) {
			double iSegLength = Math.hypot(Pathpoints.get(i).x - Pathpoints.get(i - 1).x, Pathpoints.get(i).y - Pathpoints.get(i - 1).y);
			currentSum += iSegLength;
			if(i > 1) {
				double angleChange = Math.abs(Math.atan2(Pathpoints.get(i).x - Pathpoints.get(i - 1).x, Pathpoints.get(i).y - Pathpoints.get(i - 1).y)
					- Math.atan2(Pathpoints.get(i - 1).x - Pathpoints.get(i - 2).x, Pathpoints.get(i - 1).y - Pathpoints.get(i - 2).y));
				angleChange /= iSegLength;
				angleChange *= scaleFactor;
				while(Math.abs(angleChange - 0.1) > Math.PI) {
					angleChange = 0;
					//angleChange = Math.abs(angleChange);
				}
				//System.out.println(Math.cos(angleChange));
				//angleChange *= samplenum;
				//System.out.println(angleChange * 180/Math.PI);
				
				//System.out.println(angleChange + ", " + 1/Math.cos(angleChange) + ", " + iSegLength + ", " + iSegLength/Math.cos(angleChange) + ", " + (-iSegLength+iSegLength/Math.cos(angleChange)));
				
				currentThetaAdjustedSum += iSegLength/Math.cos(angleChange);
			}else currentThetaAdjustedSum += iSegLength;
			
			//System.out.println(currentSum + ", " + currentThetaAdjustedSum + ", " + (currentThetaAdjustedSum - currentSum));
			
		}
		tAdjLength = currentThetaAdjustedSum;
		length = currentSum;
		
		LengthByTime.add(new Vector2(0, 0));
		for(int i = 1; i < Pathpoints.size(); i++) {
			double iSegLength = Math.hypot(Pathpoints.get(i).x - Pathpoints.get(i - 1).x, Pathpoints.get(i).y - Pathpoints.get(i - 1).y);
			LengthByTime.add(new Vector2((double)i/Pathpoints.size(), LengthByTime.get(LengthByTime.size() - 1).y + iSegLength));
			//System.out.println(iSegLength);
		}
	}
	static public class Waypoint{
		Vector2 position = new Vector2();
		Vector2 velocity = new Vector2();
		Vector2 acceleration = new Vector2();
		
		public Waypoint(Vector2 position){
			this.position = position;
		}
		public Waypoint(Vector2 position, Vector2 velocity){
			this.position = position;
			this.velocity = velocity;
		}
		public Waypoint(double time, Vector2 position, Vector2 velocity, Vector2 acceleration){
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
		}
		double linearVelocity() {
			return Math.sqrt(velocity.x * velocity.x + velocity.y + velocity.y);
		}
	}
}