package Kalman;

import org.ejml.simple.SimpleMatrix;

public class DCVKalman {
	KFV2 DCKalmanFilter; //KFV2(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix State, SimpleMatrix Control)
	
	public DCVKalman(double j, double b, double k, double r, double l){
		SimpleMatrix A = new SimpleMatrix(2, 2);
		SimpleMatrix B = new SimpleMatrix(2, 1);
		SimpleMatrix C = new SimpleMatrix(1, 2);
		SimpleMatrix State = new SimpleMatrix(2, 1);
		SimpleMatrix Control = new SimpleMatrix(1, 1);
		A.set(0, 0, -b/j);
		A.set(0, 1, k/j);
		A.set(1, 0, -k/j);
		A.set(1, 1, -r/l);
		
		B.set(1, 0, 1/l);
		
		C.set(0, 0, 1);
		
		DCKalmanFilter = new KFV2(A, B, C, State, Control);
	}
	void iterate(double velocity, double current) {
		DCKalmanFilter.predict();
		SimpleMatrix newState = new SimpleMatrix(2, 1);
		newState.set(0, 0, velocity);
		newState.set(1, 0, current);
		DCKalmanFilter.update(newState);
	}
	double returnVelocity() {
		return DCKalmanFilter.State.get(0, 0);
	}
	double returnCurrent() {
		return DCKalmanFilter.State.get(1, 0);
	}
}
