import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import Splines.*;
import Utility.DesmosCopyGen;
import Utility.RVC;

public class SwerveToPoint {
	final double dT = 1.0/50;
	final int pollSize = 100; //= 2000;
	final double evenDist = 10;
	double jerk = 4; // Per Second Cubed
	double maxAccel = 5; // Per Second Squared
	double maxVel = 15; // Per Second
	
	double distanceParallelP = 2;
	double distanceNormalP = 4;
	
	double angleChangeCapacity = 5; //Higher = lower capacity for turning, 0 < k < inf
	double allignmentDist = 2; //Distance
	double allignmentStrength = 10; //Double 1 < x < 10
	double convergenceDist; //Strength, at optimal swerve states
	ArrayList<Waypoint> Waypoints = new ArrayList<>();
	ArrayList<Waypoint> Trajectory = new ArrayList<>();
	SwerveDrive model = new SwerveDrive(evenDist, maxVel);
	QuinticSpline pathGenerator;
	double pathLength;
	
	
	double pathTimeOffset;//aka time start pathcreation or start of path
	Vector2 target;
	Vector2 targetVelocity;
	
	RVC accel = new RVC(jerk, maxAccel, maxVel);
	RVC decel = new RVC(jerk, maxAccel, 0);
	/*
	 	ArrayList<Waypoint> Waypoints = new ArrayList<>();
		
		Waypoints.add(new Waypoint(new Vector2(0, 0), new Vector2(10, 0), 0));
		Waypoints.add(new Waypoint(new Vector2(10, 10), new Vector2(10, 0), 1));
		Waypoints.add(new Waypoint(new Vector2(20, 15), new Vector2(10, 0), 1));
	 */
	
	
	//TRAJECTORY GENERATION
	public SwerveToPoint(Waypoint currentPose, Vector2 targetP, Vector2 targetV){
		double functionTimeStart = System.currentTimeMillis();
		
		target = targetP;
		targetVelocity = targetV;
		this.Waypoints.add(currentPose);
		this.Waypoints.add(new Waypoint(target.minus(targetVelocity.unit().multiply(allignmentDist)), targetVelocity.unit().multiply(allignmentStrength - allignmentDist))); //Allignment Point
		//this.Waypoints.add(new Waypoint(target, targetVelocity.unit())); //End
//		this.Waypoints.add(new Waypoint(target.minus(targetVelocity.unit().multiply(allignmentDist)), targetVelocity.unit().multiply(allignmentStrength - allignmentDist))); //Allignment Point
//		this.Waypoints.add(new Waypoint(target, targetVelocity.unit().multiply(allignmentStrength - allignmentDist))); //End
		pathGenerator = new QuinticSpline(Waypoints);
		pathGenerator.generateSegments();
		pathGenerator.Segments.add(new Segment(target.minus(targetVelocity.unit().multiply(allignmentDist)), target, targetVelocity.unit(), targetVelocity.unit()));
		pathGenerator.sampleSegments(pollSize);
		pathGenerator.Pathpoints.set(pathGenerator.Pathpoints.size() - 1, target);
		pathGenerator.integrateLength(angleChangeCapacity);
		this.pathLength = pathGenerator.length;
		
		DesmosCopyGen printer = new DesmosCopyGen(pathGenerator.Pathpoints);
		//printer.printWithPoints();
		
		double maxVelIterateDown = 0.2;
		int accelIters = 0;
		accel.setParams(0, 0, 0, maxVel);
		while(accel.positionTraveresed() > pathLength/2) {
			accelIters ++;
			accel.setParams(0, 0, 0, maxVel - maxVelIterateDown * accelIters);
		}
		double accelTime = accel.p4 - accel.p1;
		double cruiseTime = (pathLength - 2*accel.positionTraveresed())/(maxVel - maxVelIterateDown * accelIters);
		decel.setParams(cruiseTime + (accelTime), (maxVel - maxVelIterateDown * accelIters), 0, 0);
		
		
		double curvatureRatio = pathGenerator.length / pathGenerator.tAdjLength;
		//System.out.println(curvatureRatio);
		RVC curvedAccel = new RVC(jerk * curvatureRatio, maxAccel * curvatureRatio, maxVel);
		RVC curvedDecel = new RVC(jerk * curvatureRatio, maxAccel * curvatureRatio, 0);
		curvedAccel.setParams(0, 0, 0, (maxVel - maxVelIterateDown * accelIters));
		curvedDecel.setParams(cruiseTime * curvatureRatio, (maxVel - maxVelIterateDown * accelIters), 0, 0);
		
		accel.printProfile();
		decel.printProfile();
//		curvedAccel.printProfile();
		//curvedDecel.printProfile();
		
		//Haha, let's pretend RVC::tri doesn't exist :)
		
		//ArrayList<Vector2> discreteBaseSample = new ArrayList<>();
		ArrayList<Vector2> discreteIntegralSample = new ArrayList<>();
		double profileIntegral = 0;
		double samplingPrevVel = 0;
		for(double i = 0; i < decel.p4; i += decel.p4/(pollSize * 5)) {
			if(i < accel.p4) {
				profileIntegral += (accel.returnVelocity(i) + samplingPrevVel)/2 * decel.p4/(pollSize * 5);
				//discreteBaseSample.add(new Vector2(i, (accel.returnVelocity(i) + samplingPrevVel)/2));
				samplingPrevVel = accel.returnVelocity(i);
			}else if(i > decel.p1) {
				profileIntegral += (decel.returnVelocity(i) + samplingPrevVel)/2 * decel.p4/(pollSize * 5);
				//discreteBaseSample.add(new Vector2(i, (decel.returnVelocity(i) + samplingPrevVel)/2));
				samplingPrevVel = decel.returnVelocity(i);
			}else {
				profileIntegral += (maxVel - maxVelIterateDown * accelIters + samplingPrevVel)/2 * decel.p4/(pollSize * 5);
				//discreteBaseSample.add(new Vector2(i, (maxVel - maxVelIterateDown * accelIters + samplingPrevVel)/2));
				samplingPrevVel = maxVel - maxVelIterateDown * accelIters;
			}
			//System.out.println(i);
			discreteIntegralSample.add(new Vector2(i, profileIntegral));
		}
		
		DesmosCopyGen printer2 = new DesmosCopyGen(discreteIntegralSample);
		//printer2.printWithPoints();

		
//		System.out.println((maxVel - maxVelIterateDown * accelIters));
//		System.out.println(accel.positionTraveresed() * 2);
//		System.out.println(accel.positionTraveresed() * 2 + (decel.p1-accel.p4) * (maxVel - maxVelIterateDown * accelIters));
		for(Vector2 vec : pathGenerator.LengthByTime) {
			//vec.x *= accel.p4;
			vec.x = findXAtYDiscreteProfile(discreteIntegralSample, vec.y);
		}
		
		DesmosCopyGen printer3 = new DesmosCopyGen(pathGenerator.LengthByTime);
		printer3.printWithLine();
		
//		System.out.println(pathGenerator.LengthByTime.get(1).y);
//		System.out.println(pathLength);
		
		//now, using Waypoint class to assign time, velocity and position to each point
		
		for(int i = 0; i < pathGenerator.LengthByTime.size(); i ++) {
			Vector2 position = pathGenerator.Pathpoints.get(i);
			double angleChange = 0;
			if(i != 0 && i != pathGenerator.LengthByTime.size() - 1) angleChange = Math.abs(Math.atan2(pathGenerator.Pathpoints.get(i + 1).x - pathGenerator.Pathpoints.get(i).x, pathGenerator.Pathpoints.get(i + 1).y - pathGenerator.Pathpoints.get(i).y)
					- Math.atan2(pathGenerator.Pathpoints.get(i).x - pathGenerator.Pathpoints.get(i - 1).x, pathGenerator.Pathpoints.get(i).y - pathGenerator.Pathpoints.get(i - 1).y));
			angleChange = Math.abs(angleChange);
			Vector2 diffNormVec;
			if(i != pathGenerator.LengthByTime.size() - 1) diffNormVec = pathGenerator.Pathpoints.get(i + 1).minus(pathGenerator.Pathpoints.get(i)).unit();
			else diffNormVec = pathGenerator.Pathpoints.get(i).minus(pathGenerator.Pathpoints.get(i - 1)).unit();
			//System.out.println("(" + i + ", " + pathGenerator.Pathpoints.get(i).y + ")");
			Vector2 velocity = diffNormVec.multiply(doubleSCurveReturnVel(pathGenerator.LengthByTime.get(i).x, curvedAccel, curvedDecel) * Math.cos(angleChangeCapacity * angleChange));
			Trajectory.add(new Waypoint(position, velocity));
			if(i == 0) Trajectory.get(0).time = 0;
			else {
				double timeStep = pathGenerator.LengthByTime.get(i).x - pathGenerator.LengthByTime.get(i - 1).x;
				Trajectory.get(i).time = Trajectory.get(i - 1).time + timeStep / Math.cos(angleChangeCapacity * angleChange);
			}
			
			if(i != 0) Trajectory.get(i).acceleration = Trajectory.get(i).velocity.minus(Trajectory.get(i - 1).velocity);
		}
		for(Waypoint p : Trajectory) p.printDesmos(0.4);
		
		System.out.println("Time: "+(System.currentTimeMillis() - functionTimeStart)/1000);
	}
	double doubleSCurveReturnVel(double x, RVC acc, RVC dec) {
		if(x <= decel.p1) {
			return accel.returnVelocity(x);
		}else{
			return decel.returnVelocity(x);
		}
	}
	double findXAtYDiscreteProfile(ArrayList<Vector2> data, double y) {
		double max = data.get(data.size()-1).y;
		double min = data.get(0).y;
		
		if(y <= max && y >= min) {
			for(int i = 0; i < data.size() - 1; i ++) {
				if(y >= data.get(i).y && y <= data.get(i + 1).y) {
					double slope = (data.get(i + 1).y - data.get(i).y)/(data.get(i + 1).x - data.get(i).x);
					//y = m(x-x1) + y1
					//(y + mx1 -y1)/m = x
					return (y - data.get(i + 1).y)/slope + data.get(i + 1).x;
				}
			}
		}
		if(y > max) {
//			return data.get(data.size() - 1).x;
			double slope = (data.get(data.size() - 2).y - data.get(data.size() - 1).y)/(data.get(data.size() - 2).x - data.get(data.size() - 1).x);
			//y = m(x-x1) + y1
			//(y + mx1 -y1)/m = x
			return (y - data.get(data.size() - 1).y)/slope + data.get(data.size() - 2).x;
		}
		if(y < min) {
			double slope = (data.get(0).y - data.get(1).y)/(data.get(0).x - data.get(1).x);
			//y = m(x-x1) + y1
			//(y + mx1 -y1)/m = x
			return (y - data.get(0).y)/slope + data.get(0).x;
		}
		return Double.NaN;
	}
	
	
	
	//PATH FOLLOWING
	
	void followProfile(double time, double currentX, double currentY, double currentTheta, double positionalkP, double angularkP) {
		double profileTime = time - pathTimeOffset;
		//double velocityX = 
	}
}
