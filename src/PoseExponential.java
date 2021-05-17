import org.ejml.simple.SimpleMatrix;

public class PoseExponential {
	//https://file.tavsys.net/control/controls-engineering-in-frc.pdf 10.2
	final double rad2deg = Math.PI/180;
	SimpleMatrix GlobalPose = new SimpleMatrix(3, 1);
	SimpleMatrix RotationMatrix = new SimpleMatrix(3, 3);
	SimpleMatrix MatrixEquation = new SimpleMatrix(3, 3);
	//SimpleMatrix LocalPoseChange = new SimpleMatrix(1, 3);
	
	void UpdatePose(SimpleMatrix PoseChange) {
		//velocities must be a 3 x 1 matrix
		//Input angles in RADIANS
		
		double theta = GlobalPose.get(0, 2);// * rad2deg;
		double dtheta = PoseChange.get(0, 2);// * rad2deg;
		RotationMatrix.set(0, 0, Math.cos(theta));
		RotationMatrix.set(1, 0, -Math.sin(theta));
		RotationMatrix.set(0, 1, Math.sin(theta));
		RotationMatrix.set(1, 1, Math.cos(theta));
		RotationMatrix.set(2, 2, 1);
		if(Math.abs(dtheta) > 0.01) {
			MatrixEquation.set(0, 0, Math.sin(dtheta)/dtheta);
			MatrixEquation.set(1, 0, (Math.cos(dtheta) - 1)/dtheta);
			MatrixEquation.set(0, 1, (1 - Math.cos(dtheta))/dtheta);
			MatrixEquation.set(1, 1, Math.sin(dtheta)/dtheta);
			MatrixEquation.set(2, 2, 1);
		}else {
			MatrixEquation.set(0, 0, 1 - (dtheta * dtheta)/6);
			MatrixEquation.set(1, 0, -(dtheta) / 2);
			MatrixEquation.set(0, 1, (dtheta) / 2);
			MatrixEquation.set(1, 1, 1 - (dtheta * dtheta)/6);
			MatrixEquation.set(2, 2, 1);
		}
		GlobalPose.plus(RotationMatrix.mult(MatrixEquation).mult(PoseChange));
	}
}
