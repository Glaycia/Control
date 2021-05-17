package LQR;
import org.ejml.simple.SimpleMatrix;

public class DCVController {
	SSController DCSSC = new SSController(2, 1, 1);
	
	DCVController(double j, double b, double k, double r, double l){
		DCSSC.A.set(0, 0, -b/j);
		DCSSC.A.set(0, 1, k/j);
		DCSSC.A.set(1, 0, -k/j);
		DCSSC.A.set(1, 1, -r/l);
		
		DCSSC.B.set(1, 0, 1/l);
		
		DCSSC.C.set(0, 0, 1);
	}
	void iterate(double velocity, double current) {
		SimpleMatrix X = new SimpleMatrix(2, 1);
		X.set(0, 0, velocity);
		X.set(1, 0, current);
		DCSSC.iterate(X);
	}
}

/*
	https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
	
	(J)     moment of inertia of the rotor
	(b)     motor viscous friction constant
	(Kb)    electromotive force constant
	(Kt)    motor torque constant
	(Kb = Kt)
	(R)     electric resistance
	(L)     electric inductance
*/