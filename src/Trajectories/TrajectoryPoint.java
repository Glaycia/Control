package Trajectories;

import java.util.ArrayList;

import Splines.Vector2;
import Splines.Waypoint;

public class TrajectoryPoint {
	Vector2 position;
	Vector2 velocity;
	double linVelocity = 0;
	boolean userConstraintPoint = false;
	double timestamp;
	public TrajectoryPoint(Vector2 pos) {
		this.position = pos;
	}
	public TrajectoryPoint(Vector2 pos, Vector2 vel, double time) {
		this.position = pos;
		this.velocity = vel;
		this.timestamp = time;
	}
	
}
