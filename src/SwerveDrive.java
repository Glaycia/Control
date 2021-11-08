import org.ejml.simple.SimpleMatrix;

public class SwerveDrive {
	public SimpleMatrix Velocity = new SimpleMatrix(3, 1);
	public SimpleMatrix EquationMatrix = new SimpleMatrix(8, 3);
	public SimpleMatrix WheelSpeeds = new SimpleMatrix(8, 1);
	
	PoseExponential PoseCalculator;
	
	double velocityBudget = 10;
	double mod1t, mod2t, mod3t, mod4t;
	/**
     * <p>
     * Updates model's pose based on pose exponential
     * </p>
     *
     * @param timeElapsed based on time since last iteration, will calculate odometry
     * @param realVelocity 3x1 colvec with x,y,r velocity, from empirical data
     */
	void UpdatePose(double timeElapsed, SimpleMatrix realVelocity) {
		PoseCalculator.UpdatePose(realVelocity.scale(timeElapsed));
	}
	
	
	/**
     * <p>
     * Updates WheelSpeeds given desired x velocity, y velocity, and angular velocity
     * Uses normal cartesian system
     * </p>
     *
     * @param x desired horizontal velocity
     * @param y desired vertical velocity
     * @param r desired rotational velocity (prob CCW)
     */
	void IKinematics(double x, double y, double r) {
		SimpleMatrix NextVelocity = new SimpleMatrix(3, 1);
		NextVelocity.set(0, 0, x);
		NextVelocity.set(1, 0, y);
		NextVelocity.set(2, 0, r);
		WheelSpeeds = EquationMatrix.mult(NextVelocity);
	}
	/**
     * <p>
     * Returns desired angle of given module in radians
     * Automatically returns velocity adjusted for module's error in angle
     * Ex: if needs to be at 45deg and is at 30deg, will multiply velocity by cos(15deg)
     * 
     * Wheel 1 top left
     * Wheel 2 top right
     * Wheel 3 bot left
     * Wheel 4 bot right
     * </p>
     *
     * @param module number module you want to get the value of
     * @param modCurrentTheta current angle of the module, used to adjust velocity of module
     */
	double returnVel(int module, double modCurrentTheta) {
		module --;
		
		double angError = returnTheta(module + 1, modCurrentTheta) - modCurrentTheta;
		return Math.hypot(WheelSpeeds.get(2 * module), WheelSpeeds.get(2 * module + 1)) * Math.cos(angError);
	}
	double returnVel(int module) {
		switch(module) {
			case 1:
				return returnVel(module, mod1t);
			case 2:
				return returnVel(module, mod2t);
			case 3:
				return returnVel(module, mod3t);
			case 4:
				return returnVel(module, mod4t);
		}
		return 69420;
	}
	void setCurrentModAngle(double mod1t, double mod2t, double mod3t, double mod4t) {
		this.mod1t = mod1t;
		this.mod2t = mod2t;
		this.mod3t = mod3t;
		this.mod4t = mod4t;
	}
	double budgetFactor() {
		double max = returnVel(1, mod1t);
		max = returnVel(2, mod2t) > max ? returnVel(2, mod2t) : max;
		max = returnVel(3, mod2t) > max ? returnVel(3, mod2t) : max;
		max = returnVel(4, mod2t) > max ? returnVel(4, mod2t) : max;
		
		return max > velocityBudget ? velocityBudget/max : 1;
	}
	/**
     * <p>
     * Returns desired angle of given module in radians
     * Automatically returns closest angle to current state of module
     * 
     * Wheel 1 top left
     * Wheel 2 top right
     * Wheel 3 bot left
     * Wheel 4 bot right
     * </p>
     *
     * @param module number module you want to get the value of
     * @param modCurrentTheta current angle of the module, used to find the closest angle so you won't have to go 180deg around
     */
	double returnTheta(int module, double modCurrentTheta) {
		module --;
		double baseAngle = Math.atan2(WheelSpeeds.get(2 * module), WheelSpeeds.get(2 * module + 1));
		
		while(Math.abs(modCurrentTheta - baseAngle) > Math.PI/2)  {
			if(modCurrentTheta - baseAngle > Math.PI/2) {
				baseAngle += Math.PI/2;
			}else if(baseAngle - modCurrentTheta > Math.PI/2) {
				baseAngle -= Math.PI/2;
			}else {
				return baseAngle;
			}
		}
		return baseAngle;
	}
	double returnTheta(int module) {
		switch(module) {
			case 1:
				return returnTheta(module, mod1t);
			case 2:
				return returnTheta(module, mod2t);
			case 3:
				return returnTheta(module, mod3t);
			case 4:
				return returnTheta(module, mod4t);
		}
		return 69420;
	}
	void FKinematics() {
		Velocity = EquationMatrix.pseudoInverse().mult(WheelSpeeds);
	}
	SimpleMatrix FKinematics(double r1v, double r1t, double r2v, double r2t, double r3v, double r3t, double r4v, double r4t) {
		SimpleMatrix ColVec = new SimpleMatrix(8, 1);
		ColVec.setRow(0, 0, Math.cos(r1t) * r1v, Math.sin(r1t) * r1v, Math.cos(r2t) * r2v, Math.sin(r2t) * r2v, Math.cos(r3t) * r3v, Math.sin(r3t) * r3v, Math.cos(r4t) * r4v, Math.sin(r4t) * r4v);
		Velocity = EquationMatrix.pseudoInverse().mult(ColVec);
		return Velocity;
	}

	/**
     * <p>
     * Creates a new swerve drive class which can be used for
     * inverse kinematics, forward kinematics, and odometry
     * 
     * Wheel 1 top left
     * Wheel 2 top right
     * Wheel 3 bot left
     * Wheel 4 bot right
     * </p>
     *
     * @param r1x X position of wheel 1.
     * @param r1y Y position of wheel 1.
     * @param r2x X position of wheel 2.
     * @param r2y Y position of wheel 2.
     * @param r3x X position of wheel 3.
     * @param r3y Y position of wheel 3.
     * @param r4x X position of wheel 4.
     * @param r4y Y position of wheel 4.
     */
	public SwerveDrive(double r1x, double r1y, double r2x, double r2y, double r3x, double r3y, double r4x, double r4y){
		//WheelDisplacement is a column vector going r1y r1x r2y r2x r3y r3x r4y r4x
		//X is vertical up, Y is horizontal right
		EquationMatrix.setColumn(2, 0, -r1y, r1x, -r2y, r2x, -r3y, r3x, -r4y, r3y);
		EquationMatrix.set(0, 0, 1);
		EquationMatrix.set(1, 1, 1);
		EquationMatrix.set(2, 0, 1);
		EquationMatrix.set(3, 1, 1);
		EquationMatrix.set(4, 0, 1);
		EquationMatrix.set(5, 1, 1);
		EquationMatrix.set(6, 0, 1);
		EquationMatrix.set(7, 1, 1);
	}
	/**
     * <p>
     * Creates a new swerve drive class which can be used for
     * inverse kinematics, forward kinematics, and odometry
     * 
     * Wheel 1 top left
     * Wheel 2 top right
     * Wheel 3 bot left
     * Wheel 4 bot right
     * </p>
     *
     * @param r1x X position of wheel 1.
     * @param r1y Y position of wheel 1.
     * @param r2x X position of wheel 2.
     * @param r2y Y position of wheel 2.
     * @param r3x X position of wheel 3.
     * @param r3y Y position of wheel 3.
     * @param r4x X position of wheel 4.
     * @param r4y Y position of wheel 4.
     * @param velMax maximum velocity of each wheel
     */
	public SwerveDrive(double r1x, double r1y, double r2x, double r2y, double r3x, double r3y, double r4x, double r4y, double velMax){
		//WheelDisplacement is a column vector going r1y r1x r2y r2x r3y r3x r4y r4x
		//X is vertical up, Y is horizontal right
		EquationMatrix.setColumn(2, 0, -r1y, r1x, -r2y, r2x, -r3y, r3x, -r4y, r3y);
		EquationMatrix.set(0, 0, 1);
		EquationMatrix.set(1, 1, 1);
		EquationMatrix.set(2, 0, 1);
		EquationMatrix.set(3, 1, 1);
		EquationMatrix.set(4, 0, 1);
		EquationMatrix.set(5, 1, 1);
		EquationMatrix.set(6, 0, 1);
		EquationMatrix.set(7, 1, 1);
		
		this.velocityBudget = velMax;
	}
	/**
     * <p>
     * Creates a new swerve drive class which can be used for
     * inverse kinematics, forward kinematics, and odometry
     * </p>
     *
     * @param evenDist assuming each wheel is even distance from the center, in a square config
     */
	public SwerveDrive(double evenDist) {
		EquationMatrix.setColumn(2, 0, -evenDist, evenDist, evenDist, evenDist, -evenDist, -evenDist, evenDist, -evenDist);
		EquationMatrix.set(0, 0, 1);
		EquationMatrix.set(1, 1, 1);
		EquationMatrix.set(2, 0, 1);
		EquationMatrix.set(3, 1, 1);
		EquationMatrix.set(4, 0, 1);
		EquationMatrix.set(5, 1, 1);
		EquationMatrix.set(6, 0, 1);
		EquationMatrix.set(7, 1, 1);
	}
	/**
     * <p>
     * Creates a new swerve drive class which can be used for
     * inverse kinematics, forward kinematics, and odometry
     * </p>
     *
     * @param evenDist assuming each wheel is even distance from the center, in a square config
      	* @param velMax maximum velocity of each wheel
     */
	public SwerveDrive(double evenDist, double velMax) {
		EquationMatrix.setColumn(2, 0, -evenDist, evenDist, evenDist, evenDist, -evenDist, -evenDist, evenDist, -evenDist);
		EquationMatrix.set(0, 0, 1);
		EquationMatrix.set(1, 1, 1);
		EquationMatrix.set(2, 0, 1);
		EquationMatrix.set(3, 1, 1);
		EquationMatrix.set(4, 0, 1);
		EquationMatrix.set(5, 1, 1);
		EquationMatrix.set(6, 0, 1);
		EquationMatrix.set(7, 1, 1);

		this.velocityBudget = velMax;
	}
}