package Splines;

import java.util.ArrayList;

public class QuinticSpline {
	double samplenum = 100;
	double length = 0;
	public ArrayList<Vector2> Pathpoints = new ArrayList<>();
	public ArrayList<Splines.Waypoint> Waypoints = new ArrayList<>();
	public ArrayList<Segment> Segments = new ArrayList<>();
	
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
			}
		}
		Pathpoints.add(new Vector2(Waypoints.get(Waypoints.size() - 1).position.x, Waypoints.get(Waypoints.size() - 1).position.y));
	}
	public void sampleSegments() {
		sampleSegments(samplenum);
	}
	public void integrateLength() {
		double currentSum = 0;
		for(int i = Pathpoints.size() - 1; i > 0; i--) {
			currentSum += Math.hypot(Pathpoints.get(i).x - Pathpoints.get(i - 1).x, Pathpoints.get(i).y - Pathpoints.get(i - 1).y);
		}
		length = currentSum;
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
	static public class Segment{
		Vector2 p0 = new Vector2();
		Vector2 p1 = new Vector2();
		Vector2 v0 = new Vector2();
		Vector2 v1 = new Vector2();
		Vector2 a0 = new Vector2();
		Vector2 a1 = new Vector2();
		
		Segment(Vector2 p0, Vector2 p1){
			this.p0 = p0;
			this.p1 = p1;
		}
		Segment(Vector2 p0, Vector2 p1, Vector2 v0, Vector2 v1){
			this.p0 = p0;
			this.p1 = p1;
			this.v0 = v0;
			this.v1 = v1;
		}
		Segment(Vector2 p0, Vector2 p1, Vector2 v0, Vector2 v1, Vector2 a0, Vector2 a1){
			this.p0 = p0;
			this.p1 = p1;
			this.v0 = v0;
			this.v1 = v1;
			this.a0 = a0;
			this.a1 = a1;
		}
		
		double array[] = {0, 0, 0, 0, 0, 0};
		void updateArray(double t) {
			array[0] = 1 + t * t * t * (-10 + 15 * t - 6 * t * t);
			array[1] = t * t * t * (10 - 15 * t + 6 * t * t);
			array[2] = t + t * t * t * (-6 + 8 * t - 3 * t * t);
			array[3] = t * t * t * (-4 + 7 * t - 3 * t * t);
			array[4] = t * t * (0.5 - 1.5 * t + 1.5 * t * t - 0.5 * t * t * t);
			array[5] = t * t * t * (0.5 - 1 * t + 0.5 * t * t);
		}
		
		public Vector2 interpolate() {
			Vector2 p0s = p0.multiply(array[0]);
			Vector2 p1s = p1.multiply(array[1]);
			Vector2 v0s = v0.multiply(array[2]);
			Vector2 v1s = v1.multiply(array[3]);
			Vector2 a0s = a0.multiply(array[4]);
			Vector2 a1s = a1.multiply(array[5]);
			return p0s.plus(p1s).plus(v0s).plus(v1s).plus(a0s).plus(a1s);
		}
	}
}