package LQR;
import org.ejml.simple.SimpleMatrix;

public class DCPController {
	SSController DCSSC = new SSController(3, 1, 1);
	
	DCPController(double j, double b, double k, double r, double l){
		DCSSC.A.set(0, 1, 1);
		DCSSC.A.set(1, 1, -b/j);
		DCSSC.A.set(2, 1, k/j);
		DCSSC.A.set(1, 2, -k/j);
		DCSSC.A.set(2, 2, -r/l);
		
		DCSSC.B.set(2, 0, 1/l);
		
		DCSSC.C.set(0, 0, 1);
	}
//	void iterate(double position, double velocity, double current) {
//		SimpleMatrix X = new SimpleMatrix(3, 1);
//		X.set(0, 0, position);
//		X.set(1, 0, velocity);
//		X.set(2, 0, current);
//		DCSSC.iterate(X);
//	}
}

/*
	https://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=ControlStateSpace
	https://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=SystemModeling
	
	(J)     moment of inertia of the rotor
	(b)     motor viscous friction constant
	(Kb)    electromotive force constant
	(Kt)    motor torque constant
	(Kb = Kt)
	(R)     electric resistance
	(L)     electric inductance
*/