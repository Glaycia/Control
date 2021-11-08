package Utility;

public class RVC {
	//Responsive Velocity Controller
	
	//Constraints
	boolean velRanged; //If true, return max velocity if above and vice versa for min
	double maxVel;
	double minVel;
	
	//Measure Values
	double time;
	double pTime;
	
	double vel;
	double pVel;
	
	double acc;
	double pAcc;
	
	//For Constructing Profile
	double sign = 1;
	double accLim;
	double jerkLim;
	double targetVel;
	
	//Lines Which Make Profile
	quad test1 = new quad();
	line test2 = new line();
	quad test3 = new quad();
	
	double tp1;
	double tp2;
	double tp3;
	double tp4;
	
	//Shifted To Final Position
	quad first = new quad();
	line second = new line();
	quad third = new quad();
	
	//Integral of Final Position
	public tri int1;
	public quad int2;
	public tri int3;
	
	
	public double p1;
	public double p2;
	public double p3;
	public double p4;
	
	public RVC(double kJ, double kA) {
		this.jerkLim = kJ;
		this.accLim = kA;
		this.targetVel = 0;
	}
	public RVC(double kJ, double kA, double targetVel) {
		this.jerkLim = kJ;
		this.accLim = kA;
		this.targetVel = targetVel;
	}
	
	public void setParams(double currentTime, double currentVel, double currentAcc, double newTarget) {
		targetVel = newTarget;
		if(targetVel < currentVel) {
			sign *= -1;
			accLim *= -1;
			jerkLim *= -1;
		}
		
		
		pTime = time;
		pVel = vel;
		pAcc = acc;
		
		time = currentTime;
		vel = currentVel;
		acc = currentAcc;
		
		tp1 = 0;
		
		if(Math.abs(acc) > Math.abs(accLim) && sign(acc) == sign) {
			//Acceleration > Maximum Acceleration while acceleration faces towards target
			test1.setParabVertex(-jerkLim/2, 0, 0);
			test1.setParabVertex(-jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(acc) != sign){
			//Acceleration > Max acceleration while acceleration is opposite of target
			test1.setParabVertex(jerkLim/2, 0, 0);
			test1.setParabVertex(jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}else {
			//Acceleration is under max Acceleration, all is normal
			test1.setParabVertex(jerkLim/2, 0, 0);
			test1.setParabVertex(jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}
		
		//Determine Cases
		double vertexX = test1.vertexX();
		double vertexY = test1.findYAtX(vertexX);
		double prospectX = test1.findDerivative(accLim);
		double prospectY = test1.findYAtX(prospectX);
		
		if((Math.abs(targetVel - vel) < 0.01)) {
			//Velocity is basically same as targetVelocity
			//System.out.println("Case 0");
			test2.setPointSlope(0, 0, targetVel);
			
			tp1 = -1000000;
			tp2 = -1000000;
			tp3 = 1000000;
			tp4 = 1000000; 
		}else if(Math.abs(acc) < Math.abs(accLim) && Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY)) {
			//Case Where Acc < Limit and absolute velocity increases, Cruise Is Reached
			//System.out.println("Case 1 : Accelerate Cruise Decellerate");
			test2.setPointSlope(accLim, test1.findDerivative(accLim), test1.findYAtX(test1.findDerivative(accLim)));
			double yTransition = targetVel - (test1.findYAtX(test1.findDerivative(accLim)) - test1.findYAtX(test1.vertexX()));
			double xVert = test2.findXAtY(yTransition) + Math.abs(test1.findDerivative(accLim) - test1.vertexX());
			
			test3.setParabVertex(-jerkLim/2, xVert, targetVel);
			
			tp2 = test1.findDerivative(accLim);
			tp3 = test3.findDerivative(accLim);
			tp4 = test3.vertexX();
		}else if(Math.abs(acc) < Math.abs(accLim) && Math.abs(prospectY - vertexY) * 2 >= Math.abs(targetVel - vertexY)) {
			//Case Where Acc < Limit, Cruise Is NOT Reached
			//System.out.println("Case 2 : Accelerate Cruise ");
			double xToHalf = sign == 1 ? test1.solve(test1.a, test1.b, test1.c - (vel + targetVel)/2, 2) : test1.solve(test1.a, test1.b, test1.c - (vel + targetVel)/2, 1);
			double xToHalfNAcc = sign == 1 ? test1.solve(test1.a, 0, test1.c - (vel + targetVel)/2, 2) : test1.solve(test1.a, 0, test1.c - (vel + targetVel)/2, 1);
			
			tp2 = tp1 + xToHalf;
			tp3 = tp1 + xToHalf;
			tp4 = tp1 + (xToHalf + xToHalfNAcc);
			
			test3.setParabVertex(-jerkLim/2, (xToHalf + xToHalfNAcc), targetVel);
		}else if(Math.abs(acc) >= Math.abs(accLim) && sign(vel - targetVel) == sign(vertexY - targetVel)) {
			//Case where Acc >= Limit, Cruise Is Reached with no overshoot
			// && Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY) //Perhaps
			//System.out.println("Case 3");
			tp2 = prospectX;
			
			test2.setPointSlope(accLim, tp2, test1.findYAtX(tp2));
			
			tp3 = tp1 + test2.findXAtY(targetVel - Math.abs(vertexY - prospectY) * sign);
			tp4 = tp1 + test2.findXAtY(targetVel - Math.abs(vertexY - prospectY) * sign) + Math.abs(prospectX - vertexX);
			
			test3.setParabVertex(-jerkLim/2, tp4, targetVel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(vel - targetVel) != sign(vertexY - targetVel) && Math.abs(prospectY - vertexY) * 2 >= Math.abs(vertexY - targetVel)) {
			//Case where Acc > Limit and overshoot is required to decelerate sufficiently, yet no cruise is required
			//System.out.println("Case 4: Overshoot Without Cruising");
			double xToHalf = test1.solve(test1.a, test1.b, test1.c - (vertexY + targetVel)/2, 1);
			
			if(sign == -1) xToHalf = test1.solve(test1.a, test1.b, test1.c - (vertexY + targetVel)/2, 2);
			
			tp2 = tp1 + xToHalf;
			tp3 = tp1 + xToHalf;
			tp4 = tp1 + (xToHalf * 2 - vertexX);
			
			test3.setParabVertex(jerkLim/2, (xToHalf * 2 - vertexX), targetVel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(vel - targetVel) != sign(vertexY - targetVel) && Math.abs(prospectY - vertexY) * 2 < Math.abs(vertexY - targetVel)) {
			//Case where Acc > Limit and overshoot is required to decellerate as well as cruise
			//System.out.println("Case 5: How did we get here?");
			
			tp2 = test1.findDerivative(-accLim);
			
			test2.setPointSlope(-accLim, tp2, test1.findYAtX(tp2));
			
			tp3 = tp1 + test2.findXAtY(targetVel + (vertexY - prospectY));
			tp4 = tp1 + test2.findXAtY(targetVel + (vertexY - prospectY)) + Math.abs(prospectX - vertexX);
			
			test3.setParabVertex(jerkLim/2 * sign(acc / accLim), tp4, targetVel);
		}else {
//			System.out.println("No Case LOL");
//			
//			if(Math.abs(acc) < Math.abs(accLim)) {
//				System.out.println("Acceleration in bounds");
//			}else {
//				System.out.println("!!! Acceleration out of Bounds");
//			}
//			if(sign(vel - targetVel) != sign(vertexY - targetVel)) {
//				System.out.println("!!! Overshoots");
//			}else {
//				System.out.println("No Overshoot");
//			}
//			
//			//System.out.println(Math.abs(vertexY) < Math.abs(prospectY));
//			if(Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY)) {
//				System.out.println("Velocity Change while non constant acceleration is NOT 100% of the profile");
//			}else {
//				System.out.println("Velocity Change while non constant acceleration IS 100% of the profile");
//			}
//			System.out.println(Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY));
//			
//			System.out.println(prospectY);
//			System.out.println(prospectX);
//			System.out.println(vertexY);
//			
//			
//			System.out.println("");
//			System.out.println(2 * Math.abs(prospectY - vertexY));
//			System.out.println(Math.abs(vertexY - targetVel));
		}
		
		//Offset for time ------>
		
		first.setParabVertex(test1.a, test1.vertexX() + currentTime, test1.findYAtX(test1.vertexX()));
		second.setPointSlope(test2.m, 0 + currentTime, test2.findYAtX(0));
		third.setParabVertex(test3.a, test3.vertexX() + currentTime, test3.findYAtX(test3.vertexX()));
		
		p1 = tp1 + currentTime;
		p2 = tp2 + currentTime;
		p3 = tp3 + currentTime;
		p4 = tp4 + currentTime;
	}
//	public double positionTraveresed() {
//		//System.out.println(first.area(p1, p2));
//		//System.out.println(second.area(p2, p3));
//		//System.out.println(third.area(p3, p4));
//		return first.area(p1, p2) + second.area(p2, p3) + third.area(p3, p4);
//	}
	public double positionTraveresed() {
		return (p4 - p1)*(returnVelocity(p1) + returnVelocity(p4))/2;
	}
	public double returnVelocity(double x) {
		if(x < p1) {
			return Math.PI;
		}else if(x >= p1 && x < p2) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return first.findYAtX(x);
			}
		}else if(x >= p2 && x < p3) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return second.findYAtX(x);
			}
		}else if(x >= p3 && x < p4) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return third.findYAtX(x);
			}
		}else {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return targetVel;
			}
		}
	}
	
	public void discoverIntegral(double offset) {
		int1 = new tri(first, offset);
		int1.x1 = p1;
		int1.x2 = p2;
		int2 = new quad();
		int2.setParab(second.m/2, second.b, int1.yAtX(p2));
		int2.x1 = p2;
		int2.x2 = p3;
		int3 = new tri(third, 0);
		int3.d = int2.findYAtX(p3) - int3.yAtX(p3);
		int3.x1 = p3;
		int3.x2 = p4;
		
		int1.print();
		System.out.println(int2.a + "x^{2} + " + int2.b + "x + " + int2.c + " = y \\left\\{" + p2 + "<x<" + p3 + "\\right\\}");
		int3.print();
	}
	
	//kV * velocity + kA * acceleration + kPV * (measuredVelocity - velocity) + kPP * (measuredPosition - position)
	
	void printTestProfile() {
		System.out.println(test1.a + "x^{2} + " + test1.b + "x + " + test1.c + " = y \\left\\{" + tp1 + "<x<" + tp2 + "\\right\\}"); //x^{2}
		System.out.println("y = " + test2.m + "(x - " + 0 + ") + " + test2.b + " \\left\\{" + tp2 + "<x<" + tp3 + "\\right\\}");
		System.out.println(test3.a + "x^{2} + " + test3.b + "x + " + test3.c + " = y \\left\\{" + tp3 + "<x<" + tp4 + "\\right\\}");
	}
	public void printProfile() {
		System.out.println(first.a + "x^{2} + " + first.b + "x + " + first.c + " = y \\left\\{" + p1 + "<x<" + p2 + "\\right\\}"); //x^{2}
		System.out.println("y = " + second.m + "(x - " + 0 + ") + " + second.b + " \\left\\{" + p2 + "<x<" + p3 + "\\right\\}");
		System.out.println(third.a + "x^{2} + " + third.b + "x + " + third.c + " = y \\left\\{" + p3 + "<x<" + p4 + "\\right\\}");
	}
	double sign(double input) {
		return Math.abs(input)/input;
	}
	
	public class tri{
		double a;
		double b;
		double c;
		double d;
		
		double x1;
		double x2;
		
		public tri(quad p, double offset) {
			this.a = p.a/3;
			this.b = p.b/2;
			this.c = p.c;
			this.d = offset;
			this.x1 = p.x1;
			this.x2 = p.x2;
		}
		
		public double yAtX(double x) {
			return a * x * x * x + b * x * x + c * x + d;
		}
		public double xAtY(double y) {
			double na = b/a;
			double nb = c/a;
			double nc = d/a;
			
			double adiv3 = a/3;
			double Q = (3 * nb - na * na)/9;
			double Qcb = Q * Q * Q;
			double R = (9 * na * nb - 27 * nc - 2 * na * na * na)/54;
			double Rsq = R * R;
			double D = Qcb + Rsq;
			if (D < 0.0)
			{
			// Three unequal real roots.
			//nRoots = 3;
			double theta = Math.acos (R / Math.sqrt (-Qcb));
			double SQRT_Q = Math.sqrt (-Q);
//			x1 = 2.0 * SQRT_Q * Math.cos (theta/3.0) - a/3;
//			x2 = 2.0 * SQRT_Q * Math.cos ((theta+2*Math.PI)/3.0) - adiv3;
//			x3 = 2.0 * SQRT_Q * Math.cos ((theta+4*Math.PI)/3.0) - adiv3;
			return 2.0 * SQRT_Q * Math.cos (theta/3.0) - a/3;
			}
		else if (D > 0.0)
			{
			// One real root.
			//nRoots = 1;
			double SQRT_D = Math.sqrt (D);
			double S = Math.cbrt (R + SQRT_D);
			double T = Math.cbrt (R - SQRT_D);
//			x1 = (S + T) - a_over_3;
//			x2 = Double.NaN;
//			x3 = Double.NaN;
			return S+T-adiv3;
			}
		else
			{
			// Three real roots, at least two equal.
			//nRoots = 3;
			double CBRT_R = Math.cbrt (R);
//			x1 = 2*CBRT_R - adiv3;
//			x2 = x3 = CBRT_R - adiv3;
			return 2*CBRT_R - adiv3;
			}
		}
		
		public void print() {
			System.out.println(a + "x^{3} + " + b + "x^{2} + " + c + "x + " + d + " = y \\left\\{" + x1 + "<x<" + x2 + "\\right\\}");
		}
	}
	class quad {
		double a;
		double b;
		double c;
		
		double x1;
		double x2;
		double x1() {
			double termSum = a * Math.pow(x1, 3)/3 + b * Math.pow(x1, 2)/2 + x1 * c;
			return Math.abs(termSum/(-1));
		}
		double x2() {
			double termSum = a * Math.pow(x2, 3)/3 + b * Math.pow(x2, 2)/2 + x2 * c;
			return Math.abs(termSum/(-1));
		}
		double area() {
			return x2() - x1();
		}
		double area(double x1, double x2) {
			this.x1 = x1;
			this.x2 = x2;
			return x2() - x1();
		}
		void setParabSlopeAtPoint(double a, double slope, point K) {
			//derivative = 2a(x-vertex)
			//x - derivative / (2a) = vertex
			double xVert = K.x - slope / (2 * a);
			setParabVertex(a, xVert, K.y + a * Math.pow((K.x - xVert), 2));
		}
		void setParab2Points(point vertex, point reference) {
			double xDiff = reference.x - vertex.x;
			double yDiff = reference.y - vertex.y;
			double a = yDiff/(xDiff * xDiff);
			setParabVertex(a, vertex.x, vertex.y);
		}
		void setParabVertex(double tA, double tX, double tY) {
			//y = a(x-h)^2 + k
			//y = ax^2 - 2hx + h * h + k
			a = tA;
			b = - tA * 2 * tX;
			c = tA * tX * tX + tY;
		}
		void setParab(double tA, double tB, double tC) {
			a = tA;
			b = tB;
			c = tC;
		}
		double vertexX () {
			return -b / (2 * a);
		}
		double derivative(double x) {
			return 2 * a * (x - vertexX());
		}
		double findDerivative(double derivative) {
			return derivative / (2 * a) + vertexX();
		}
		double findYAtX(double x) {
			return a * x * x + b * x + c;
		}
		double solve(double a, double b, double c, int root) {
			if(root == 1) {
				return ((-b - Math.sqrt(b*b-4*a*c))/(2*a));
			}else if(root == 2) {
				return ((-b + Math.sqrt(b*b-4*a*c))/(2*a));
			}else {
				return 1/0;
			}
		}
		double solve(int root) {
			if(root == 1) {
				return ((-b - Math.sqrt(b*b-4*a*c))/(2*a));
			}else if(root == 2) {
				return ((-b + Math.sqrt(b*b-4*a*c))/(2*a));
			}else {
				return 1/0;
			}
		}
		//https://www.mathsisfun.com/calculus/integration-rules.html
	}
	class line{
		double b;
		double m;
		
		double x1;
		double x2;
		
		//Integral = mx^2/2 + bx
		double x1() {
			double termSum = m * Math.pow(x1, 2)/2 + x1 * b;
			return Math.abs(termSum/(-1));
		}
		double x2() {
			double termSum = m * Math.pow(x2, 2)/2 + x2 * b;
			return Math.abs(termSum/(-1));
		}
		double area() {
			return x2() - x1();
		}
		double area(double x1, double x2) {
			this.x1 = x1;
			this.x2 = x2;
			return x2() - x1();
		}
		void setPointSlope(double tM, double tX, double tY) {
			//y - tY = m (x - tX)
			//y = mx - mtX + tY
			m = tM;
			b = -m * tX + tY;
		}
		void setSlopeIntercept(double tM, double tB) {
			m = tM;
			b = tB;
		}
		double findYAtX(double x) {
			return m * x + b;
		}
		double findXAtY(double y) {
			return (y - b) / m;
		}
		//https://www.mathsisfun.com/calculus/integration-rules.html
	}
	class point{
		point(double x, double y){
			this.x = x;
			this.y = y;
		}
		point(){
			x = 0;
			y = 0;
		}
		double x;
		double y;
	}
}