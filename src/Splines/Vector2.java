package Splines;

public class Vector2 {
	public double x;
	public double y;
	
	public Vector2(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Vector2() {
		this.x = 0;
		this.y = 0;
	}
	
	double length() {
		return Math.sqrt(x*x + y*y);
	}
	
	public Vector2 multiply(double scale) {
		return new Vector2(x * scale, y * scale);
	}
	public Vector2 plus(Vector2 vector) {
		return new Vector2(x + vector.x, y + vector.y);
	}
	public Vector2 minus(Vector2 vector) {
		return new Vector2(x - vector.x, y - vector.y);
	}
	public Vector2 unit() {
		return new Vector2(x/length(), y/length());
	}

}
