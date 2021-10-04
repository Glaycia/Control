package iLQR;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class DifferentialDriveEnv implements Environment{
	int xDim = 3;
	int uDim = 2;
	
	int horizon = 100;
	double dT = 1/10;
	double robotRadius = 2;
	double wheelDist = 1/Math.PI; //Change, is hypoth rn
	
	ArrayList<SimpleMatrix> obsArray;
	
	//Cost Arrays
	double obstacleScaleFactor;
	
	//Trajectory Data
	ArrayList<SimpleMatrix> L;
	ArrayList<SimpleMatrix> l;
	
	public DifferentialDriveEnv() {	
		for(int i = 0; i < horizon; i ++) L.add(new SimpleMatrix(uDim, xDim));
	}
	
	public SimpleMatrix obstacle(double x, double y, double radius, double weight) {
		SimpleMatrix toReturn = new SimpleMatrix(4, 1);
		toReturn.set(0, x);
		toReturn.set(1, y);
		toReturn.set(2, radius);
		toReturn.set(3, weight);
		return toReturn;
	}
	public void initObstacles() {
		obsArray.add(obstacle(6, 9, 1, 1));
		obsArray.add(obstacle(20, 3, 1, 1));
	}
	public double obstacleCost(SimpleMatrix state) {
		double cost = 0;
		for(int i = 0; i < obsArray.size(); i++) cost += obsArray.get(i).get(3) * Math.exp(-(Math.hypot(state.get(0) - obsArray.get(i).get(0), state.get(1) - obsArray.get(i).get(1) - robotRadius - obsArray.get(i).get(2))));
		return cost * obstacleScaleFactor;
	}
	public double costToGo(int horizonStep) {
		return 4;
	}
	public double actionCost(int horizonStep) {
		return 4;
	}
	public SimpleMatrix continuous(SimpleMatrix state, SimpleMatrix control) { //returns column vector, xDot;
		SimpleMatrix returnVector = new SimpleMatrix(3, 1);
		
		returnVector.set(0, (control.get(0) + control.get(1)) * (Math.cos(state.get(2)/2)));
		returnVector.set(1, (control.get(0) + control.get(1)) * (Math.sin(state.get(2)/2)));
		returnVector.set(2, (control.get(1) + control.get(0))/wheelDist);
		
		return returnVector;
	}
	public SimpleMatrix dynamics(SimpleMatrix state, SimpleMatrix control) { //returns column vector, x+1
		SimpleMatrix k1 = continuous(state, control);
		SimpleMatrix k2 = continuous(state.plus(k1.scale(dT/2)), control);
		SimpleMatrix k3 = continuous(state.plus(k2.scale(dT/2)), control);
		SimpleMatrix k4 = continuous(state.plus(k3.scale(dT)), control);
		return state.plus((k1.plus(k2.scale(2)).plus(k3.scale(2).plus(k4))).scale(dT/6));
	}
}
