package iLQR;

import org.ejml.simple.SimpleMatrix;

public interface Environment{
	//int horizon; //size
	public double costToGo(int horizonStep);
	public double actionCost(int horizonStep);
	public SimpleMatrix dynamics(SimpleMatrix state, SimpleMatrix control); //returns column vector;
}
