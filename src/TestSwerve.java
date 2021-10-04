
public class TestSwerve {

	public static void main(String[] args) {
		//1  2
		//3  4
		//WheelDisplacement is a column vector going r1y r1x r2y r2x r3y r3x r4y r4x
		double centerDist = 0.71;
		SwerveDrive SwKinem = new SwerveDrive(centerDist, 10);
		SwKinem.IKinematics(22, 15, 0.5);
		
		final double toRad = 180/Math.PI;
		
		SwKinem.setCurrentModAngle(0, 0, 0, 0);
		System.out.println("Vel M1:   " + SwKinem.returnVel(1) * SwKinem.budgetFactor());
		System.out.println("Theta M1: " + SwKinem.returnTheta(1) * toRad);
		System.out.println("Vel M2:   " + SwKinem.returnVel(2) * SwKinem.budgetFactor());
		System.out.println("Theta M2: " + SwKinem.returnTheta(2) * toRad);
		System.out.println("Vel M3:   " + SwKinem.returnVel(3) * SwKinem.budgetFactor());
		System.out.println("Theta M3: " + SwKinem.returnTheta(3) * toRad);
		System.out.println("Vel M4:   " + SwKinem.returnVel(4) * SwKinem.budgetFactor());
		System.out.println("Theta M4: " + SwKinem.returnTheta(4) * toRad);
	}

}