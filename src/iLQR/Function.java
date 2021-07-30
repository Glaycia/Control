package iLQR;

import org.ejml.simple.SimpleMatrix;

public abstract class Function {
//	double function(SimpleMatrix a, SimpleMatrix b) {
//		return 0;
//	}
	SimpleMatrix function(SimpleMatrix a, SimpleMatrix b) {
		return new SimpleMatrix(1, 1);
	}
}
