package Splines;

public class Waypoint{
	public double time = 0;
	public Vector2 position = new Vector2();
	public Vector2 velocity = new Vector2();
	public Vector2 acceleration = new Vector2();
	
	public boolean userConstrainVelocity = false;
	public double intendedVelocity;
	
	public boolean swerveMode;
	public double swerveAngle; //Degrees Probably, this is only to feed into the controller
	
	public Waypoint(Vector2 position){
		this.position = position;
	}
	public Waypoint(Vector2 position, Vector2 velocity){
		this.position = position;
		this.velocity = velocity;
	}
	public Waypoint(Vector2 position, Vector2 velocity, Vector2 acceleration){
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}
	public Waypoint(Vector2 position, double rVelocity){
		this.position = position;
		this.userConstrainVelocity = true;
		this.intendedVelocity = rVelocity;
	}
	public Waypoint(Vector2 position, Vector2 velocity, double rVelocity){
		this.position = position;
		this.velocity = velocity;
		this.userConstrainVelocity = true;
		this.intendedVelocity = rVelocity;
	}
	public Waypoint(Vector2 position, Vector2 velocity, Vector2 acceleration, double rVelocity){
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.userConstrainVelocity = true;
		this.intendedVelocity = rVelocity;
	}
	
	public void printDesmos(double velocityFactor) {	
		String input = ("((1-t)(" + position.x + ") + t(" + (position.x + velocity.x * velocityFactor) + "), (1-t)(" + position.y + ") + t(" + (position.y + velocity.y * velocityFactor) + "))");
        System.out.println(input);
	}
	/*
	public Waypoint(double time, Vector2 position){
		this.time = time;
		this.position = position;
	}
	public Waypoint(double time, Vector2 position, Vector2 velocity){
		this.time = time;
		this.position = position;
		this.velocity = velocity;
	}
	public Waypoint(double time, Vector2 position, Vector2 velocity, Vector2 acceleration){
		this.time = time;
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}
	*/
	double linearVelocity() {
		return Math.sqrt(velocity.x * velocity.x + velocity.y + velocity.y);
	}
}
