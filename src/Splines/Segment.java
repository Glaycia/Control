package Splines;

public class Segment{
	Vector2 p0 = new Vector2();
	Vector2 p1 = new Vector2();
	Vector2 v0 = new Vector2();
	Vector2 v1 = new Vector2();
	Vector2 a0 = new Vector2();
	Vector2 a1 = new Vector2();
	
	public Segment(Vector2 p0, Vector2 p1){
		this.p0 = p0;
		this.p1 = p1;
	}
	public Segment(Vector2 p0, Vector2 p1, Vector2 v0, Vector2 v1){
		this.p0 = p0;
		this.p1 = p1;
		this.v0 = v0;
		this.v1 = v1;
	}
	public Segment(Vector2 p0, Vector2 p1, Vector2 v0, Vector2 v1, Vector2 a0, Vector2 a1){
		this.p0 = p0;
		this.p1 = p1;
		this.v0 = v0;
		this.v1 = v1;
		this.a0 = a0;
		this.a1 = a1;
	}
	
	double array[] = {0, 0, 0, 0, 0, 0};
	void updateArray(double t) {
		array[0] = 1 + t * t * t * (-10 + 15 * t - 6 * t * t);
		array[1] = t * t * t * (10 - 15 * t + 6 * t * t);
		array[2] = t + t * t * t * (-6 + 8 * t - 3 * t * t);
		array[3] = t * t * t * (-4 + 7 * t - 3 * t * t);
		array[4] = t * t * (0.5 - 1.5 * t + 1.5 * t * t - 0.5 * t * t * t);
		array[5] = t * t * t * (0.5 - 1 * t + 0.5 * t * t);
	}
	
	public Vector2 interpolate() {
		Vector2 p0s = p0.multiply(array[0]);
		Vector2 p1s = p1.multiply(array[1]);
		Vector2 v0s = v0.multiply(array[2]);
		Vector2 v1s = v1.multiply(array[3]);
		Vector2 a0s = a0.multiply(array[4]);
		Vector2 a1s = a1.multiply(array[5]);
		return p0s.plus(p1s).plus(v0s).plus(v1s).plus(a0s).plus(a1s);
	}
}
