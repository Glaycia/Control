package Trajectories;

import java.util.ArrayList;

import Splines.QuinticSpline;
import Splines.Vector2;
import Splines.Waypoint;

public class TrajectoryGenerator {
	double maxAccel;
	double userCapVel;
	
	QuinticSpline path;
	ArrayList<TrajectoryPoint> pathpoints = new ArrayList<>();
	
	
	TrajectoryGenerator(double maxAcceleration, double userSetVelocity) {
		this.maxAccel = maxAcceleration;
		this.userCapVel = userSetVelocity;
	}
	public TrajectoryGenerator(double maxAcceleration, double userSetVelocity, ArrayList<Waypoint> constraints) {
		this.maxAccel = maxAcceleration;
		this.userCapVel = userSetVelocity;
		path = new QuinticSpline(constraints);
		path.generateSegments();
		path.sampleSegments();
		pathpoints = convertToPath(path.Waypoints, path.Pathpoints);
	}
	public ArrayList<TrajectoryPoint> convertToPath(ArrayList<Waypoint> constraints, ArrayList<Vector2> data){
		ArrayList<TrajectoryPoint> path = new ArrayList<>();
		//System.out.println("Converting...");
		for(int i = 0; i < data.size(); i++) {
			path.add(new TrajectoryPoint(data.get(i)));
		}
		//System.out.println("Array Size: " + path.size());
		for(int i = 0; i < constraints.size(); i++) {
			for(int j = 0; j < path.size() ; j++) {
				if(path.get(j).userConstraintPoint == false && constraints.get(i).userConstrainVelocity == true && (path.get(j).position.x == constraints.get(i).position.x) && (path.get(j).position.y == constraints.get(i).position.y)) {
					path.get(j).linVelocity = constraints.get(i).intendedVelocity;
					path.get(j).userConstraintPoint = true;
					System.out.println("Waypoint Constraint Detected, Intended Velocity: " + path.get(j).linVelocity);
				}
			}
		}
		return path;
	}
	void ForwardPass() {
		double currentVelocity = pathpoints.get(0).linVelocity;
		for(int i = 0; i < pathpoints.size() - 1; i++) {
			  double segLength = Math.hypot(pathpoints.get(i).position.x - pathpoints.get(i + 1).position.x, pathpoints.get(i).position.y - pathpoints.get(i + 1).position.y);
			  double hypoTime = segLength/(currentVelocity + maxAccel/2);
			  
			  if(pathpoints.get(i).userConstraintPoint) {
				  currentVelocity = minimumBP(userCapVel, currentVelocity + hypoTime * maxAccel, pathpoints.get(i).linVelocity);
				  //System.out.println("FP Waypoint Constraint Detected, Intended Velocity: " + minimumBP(userCapVel, currentVelocity + hypoTime * maxAccel, pathpoints.get(i).linVelocity));
				  //System.out.println("FP Waypoint Constraint Detected, Intended Constraint Velocity: " + pathpoints.get(i).linVelocity);
			  }else {
				  currentVelocity = minimum(userCapVel, currentVelocity + hypoTime * maxAccel);
			  }
			  if(!pathpoints.get(i + 1).userConstraintPoint) {
				  pathpoints.get(i + 1).linVelocity = currentVelocity;
			  }else {
				  pathpoints.get(i + 1).linVelocity = minimum(userCapVel, pathpoints.get(i + 1).linVelocity);
				  //System.out.println("FP Waypoint Constraint Detected, Set Velocity: " + minimum(userCapVel, pathpoints.get(i).linVelocity));
			  }
		}
	}
	void BackwardsPass() {
		double currentVelocity = pathpoints.get(pathpoints.size() - 1).linVelocity;
		System.out.println("Backpass Initial Velocity: " + currentVelocity);
		for(int i = pathpoints.size() - 1; i  > 0; i--) {
			  double segLength = Math.hypot(pathpoints.get(i).position.x - pathpoints.get(i - 1).position.x, pathpoints.get(i).position.y - pathpoints.get(i - 1).position.y);
			  double hypoTime = segLength/(currentVelocity + maxAccel/2);
			  
			  currentVelocity = minimumBP(userCapVel, currentVelocity + hypoTime * maxAccel, pathpoints.get(i - 1).linVelocity);
			  pathpoints.get(i - 1).linVelocity = currentVelocity;
		}
	}
	void CalculateTime() {
		CalculateTime(0);
	}
	void CalculateTime(double timeOffset) {
		pathpoints.get(0).timestamp = timeOffset;
		for(int i = 1; i < pathpoints.size(); i++) {
			double segLength = Math.hypot(pathpoints.get(i).position.x - pathpoints.get(i - 1).position.x, pathpoints.get(i).position.y - pathpoints.get(i - 1).position.y);
			pathpoints.get(i).timestamp = pathpoints.get(i - 1).timestamp + Math.abs(segLength/(pathpoints.get(i - 1).linVelocity + maxAccel/2));
			//System.out.println(pathpoints.get(i).timestamp);
		}
	}
	double minimum(double userCapVel, double accelCapVel) {
		if(Math.abs(userCapVel) < Math.abs(accelCapVel)) {
			return userCapVel;
		}else {
			return accelCapVel;
		}
	}
	double minimumBP(double userCapVel, double accelCapVel, double previousVelocityProfile) {
		if(Math.abs(userCapVel) < Math.abs(accelCapVel) && Math.abs(userCapVel) < Math.abs(previousVelocityProfile)) {
			return userCapVel;
		}else if(Math.abs(accelCapVel) < Math.abs(userCapVel) && Math.abs(accelCapVel) < Math.abs(previousVelocityProfile)){
			return accelCapVel;
		}else {
			return previousVelocityProfile;
		}
	}
}
