package iLQR;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class DiffDriveTest {

	public static void main(String[] args) {
		SimpleMatrix initState = new SimpleMatrix(3, 1);
		SimpleMatrix initControl = new SimpleMatrix(2, 1);
		
		SimpleMatrix defaultTargetState = new SimpleMatrix(3, 1);
		SimpleMatrix defaultControlState = new SimpleMatrix(2, 1);
		
		defaultTargetState.set(0, 10);
		defaultTargetState.set(1, 10);
		defaultTargetState.set(2, 0);
		
		defaultControlState.set(0, 0.5);
		defaultControlState.set(1, 0.5);
		
		double skf = 2;
		SimpleMatrix Q = new SimpleMatrix(3, 3); //Scaling for State Error
		SimpleMatrix R = new SimpleMatrix(2, 2); //Scaling for Control Error
		double q = 1; //Scaling for distance from obstacles
		
		Q.set(0, 0, 1);
		Q.set(1, 1, 1);
		Q.set(2, 2, 1/(5 * 5));
		
		R.set(0, 0, 1/(0.1 * 0.1) * skf);
		R.set(1, 1, 1/(0.1 * 0.1) * skf);
		
		ArrayList<SimpleMatrix> obstacles = new ArrayList<>();
		
		obstacles.add(Obstacle(2, 2));
		obstacles.add(Obstacle(3, 2));
		obstacles.add(Obstacle(6, 3));
		obstacles.add(Obstacle(4, 7));
		obstacles.add(Obstacle(9, 8));
		
		DiffDriveILQR trajOpt = new DiffDriveILQR(initState, initControl, defaultTargetState, defaultControlState, obstacles, Q, R, q, obstacles);
		
		trajOpt.iterativeLQR(100, defaultTargetState, defaultControlState, 1000);
	}
	static SimpleMatrix Obstacle(double x, double y) {
		SimpleMatrix Obstacle = new SimpleMatrix(2, 1);
		Obstacle.set(0, x);
		Obstacle.set(1, y);
		return Obstacle;
	}
}
