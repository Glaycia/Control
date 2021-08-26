package LQR;
import org.ejml.simple.SimpleMatrix;

public class DCVController {
	public SSController DCSSC = new SSController(2, 1, 1);
	
	double j, b, k, r, l;
	
	public DCVController(double j, double b, double k, double r, double l){
		this.j = j;
		this.b = b;
		this.k = k;
		this.r = r;
		this.l = l;
		
		DCSSC.A.set(0, 0, -b/j);
		DCSSC.A.set(0, 1, k/j);
		DCSSC.A.set(1, 0, -k/j);
		DCSSC.A.set(1, 1, -r/l);
		
		DCSSC.B.set(1, 0, 1/l);
		
		DCSSC.C.set(0, 0, 1);
	}
	
	public void GainSetter(double maxVelError, double maxCurrentError, double maxVoltage, double stateImportance) {
		SimpleMatrix StateMaxError = new SimpleMatrix(2, 1);
		SimpleMatrix ControlMax = new SimpleMatrix(1, 1);
		StateMaxError.set(0, 0, maxVelError);
		StateMaxError.set(1, 0, maxCurrentError);
		ControlMax.set(0, 0, maxVoltage);
		LQRPicker costPicker = new LQRPicker(StateMaxError, ControlMax, stateImportance);
		DCSSC.SetLQR(costPicker.Q, costPicker.R);
	}
	public double returnVoltage(double currentVelocity, double desiredVelocity) {
		return returnVoltage(currentVelocity, 0, desiredVelocity, 0);
	}
	public double returnVoltage(double currentVelocity, double currentCurrent, double desiredVelocity, double desiredCurrent) {
		SimpleMatrix CurrentState = new SimpleMatrix(2, 1);
		SimpleMatrix DesiredState = new SimpleMatrix(2, 1);
		
		CurrentState.set(0, 0, currentVelocity);
		CurrentState.set(0, 1, currentCurrent);
		DesiredState.set(0, 0, desiredVelocity);
		DesiredState.set(0, 1, desiredCurrent);
		
		DCSSC.update(CurrentState, DesiredState);
		
		return DCSSC.computeOutput().get(0, 0);
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