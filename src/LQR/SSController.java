package LQR;

import org.ejml.simple.*;
public class SSController {
	int states;
	int inputs;
	int outputs;
	
	//[A, B] * [B, C] = [A, C]
	public SimpleMatrix A; //System Matrix (States x States)
	public SimpleMatrix B; //Input Matrix (States x Inputs)
	public SimpleMatrix C; //Output Matrix (Outputs x States)
	public SimpleMatrix D; //Feedthrough Matrix (Inputs x States)
	
	public SimpleMatrix K; //Controller Gain Matrix
	
	public SimpleMatrix X; //State Vector (States x 1)
	public SimpleMatrix U; //Input Vector (Inputs x 1)
	public SimpleMatrix Y; //Output Vector(Outputs x 1)
	public SimpleMatrix R; //Reference Vector(States x 1)
	
	SSController(int states, int inputs, int outputs){
		this.states = states;
		this.inputs = inputs;
		this.outputs = outputs;
		
		A = new SimpleMatrix(states, states);
		B = new SimpleMatrix(states, inputs);
		C = new SimpleMatrix(outputs, states);
		D = new SimpleMatrix(inputs, states);
		
		X = new SimpleMatrix(states, 1);
		U = new SimpleMatrix(inputs, 1);
		Y = new SimpleMatrix(outputs, 1);
		R = new SimpleMatrix(states, 1);
		
		//Calculate LQR Somewhere
	}
	void SetLQR(SimpleMatrix QCost, SimpleMatrix RCost) {
		LQRFactory LQRGen = new LQRFactory();
		LQRGen.A = this.A;
		LQRGen.B = this.B;
		LQRGen.Q = QCost;
		LQRGen.R = RCost;
		//LQRGen.DARERicattiArimotoPotter();
		LQRGen.DAREIteration(1000, 0.001);
		LQRGen.computeK();
		this.K = LQRGen.K;
	}
	void update(SimpleMatrix newState) {
		X = newState;
	}
	void update(SimpleMatrix newState, SimpleMatrix newReference) {
		X = newState;
		R = newReference;
	}
	SimpleMatrix computeOutput() {
		SimpleMatrix Control = K.mult(R.minus(X));
		return Control;
	}
//	void iterate(SimpleMatrix X) {
//		SimpleMatrix X_dot = X.mult(A.minus(B.mult(K))).plus(B.mult(K).mult(R));
//		//Xdot = X(A-BK)+BKR
//		Y = X.mult(C.minus(D.mult(K))).plus(D.mult(K).mult(R));
//		//Y = X(C-DK)+DKR
//		
//		X = X.plus(X_dot);
//	}
}
