import org.ejml.simple.SimpleMatrix;

public class SwerveDrive {
	SimpleMatrix Velocity = new SimpleMatrix(3, 1);
	SimpleMatrix EquationMatrix = new SimpleMatrix(8, 3);
	SimpleMatrix WheelSpeeds = new SimpleMatrix(8, 1);
	
	PoseExponential PoseCalculator;
	
	void UpdatePose(double timeElapsed) {
		PoseCalculator.UpdatePose(Velocity.scale(timeElapsed));
	}
	void IKinematics(SimpleMatrix NextVelocity) {
		WheelSpeeds = EquationMatrix.mult(NextVelocity);
	}
	void FKinematics() {
		Velocity = EquationMatrix.pseudoInverse().mult(WheelSpeeds);
	}
	SwerveDrive(SimpleMatrix WheelDisplacement){
		//WheelDisplacement is a column vector going r1y r1x r2y r2x r3y r3x r4y r4x
		//X is vertical up, Y is horizontal right
		EquationMatrix.combine(2, 0, WheelDisplacement);
		EquationMatrix.set(0, 0, 1);
		EquationMatrix.set(1, 1, 1);
		EquationMatrix.set(2, 0, 1);
		EquationMatrix.set(3, 1, 1);
		EquationMatrix.set(4, 0, 1);
		EquationMatrix.set(5, 1, 1);
		EquationMatrix.set(6, 0, 1);
		EquationMatrix.set(7, 1, 1);
	}
}
