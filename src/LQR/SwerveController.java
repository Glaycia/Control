package LQR;
import org.ejml.simple.SimpleMatrix;

public class SwerveController {
	SSController SwerveSSC = new SSController(3, 3, 3);
	
	SwerveController(SimpleMatrix Pose){
		SwerveSSC.X.set(Pose);
		SwerveSSC.B.set(SimpleMatrix.diag(1));
		SwerveSSC.C.set(SimpleMatrix.diag(1));
	}
	void iterate(double theta, SimpleMatrix ChassisVelocity, SimpleMatrix ReferenceState) {
		//Theta In Radians
//		SimpleMatrix RotationMatrix = new SimpleMatrix(3, 3);
//		RotationMatrix.set(0, 0, Math.cos(theta));
//		RotationMatrix.set(1, 0, Math.sin(theta));
//		RotationMatrix.set(0, 1, -Math.sin(theta));
//		RotationMatrix.set(1, 1, Math.cos(theta));
//		RotationMatrix.set(2, 2, 1);
		
		//Iterate from PoseExp Odometry
		SwerveSSC.iterate(ReferenceState);
		
	}
}
