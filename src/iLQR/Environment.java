package iLQR;

import org.ejml.simple.SimpleMatrix;

public interface Environment{
	//int horizon; //size
	public int getXDim();
	public int getUDim();
	public int getHorizonLen();
	
	public SimpleMatrix avHessianXX(SimpleMatrix x, SimpleMatrix u, int iter);
	public SimpleMatrix avHessianXU(SimpleMatrix x, SimpleMatrix u, int iter);
	public SimpleMatrix avHessianUU(SimpleMatrix x, SimpleMatrix u, int iter);
	public SimpleMatrix avJacobianX(SimpleMatrix x, SimpleMatrix u, int iter);
	public SimpleMatrix avJacobianU(SimpleMatrix x, SimpleMatrix u, int iter);
	public SimpleMatrix dynJacobianX(SimpleMatrix x, SimpleMatrix u);
	public SimpleMatrix dynJacobianU(SimpleMatrix x, SimpleMatrix u);
	
	//public SimpleMatrix 
	public double costToGo(SimpleMatrix state, SimpleMatrix control, int horizonStep);
	public double actionCost(SimpleMatrix state, SimpleMatrix control, int horizonStep);
	public SimpleMatrix dynamics(SimpleMatrix state, SimpleMatrix control); //returns column vector;
}
