package PID;

public abstract class PIDBody {
	double currentState;
	double previousState;
	double currentTime;
	double previousTime;
	double setpointState;
	double derivative;
	double integral;
	
	double kP;
	double kI;
	double kD;
	double iMax = 0; //Integral max, zero if no max;
	double integralRange = 0; //Maximum error before integral will increase, zero if no range;
	
	double minOut = 0; //Minimum output of controller, zero if no minimum;
	double maxOut = 0; //Maximum output of controller, zero if no maximum;
	
	void update(double input, double time) {
		update(input, this.setpointState);
	}
	void update(double input, double time, double newSetpoint) {
		currentState = input;
		currentTime = time;
		
		
	}
}
