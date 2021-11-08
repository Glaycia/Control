package iLQR;

import java.util.ArrayList;

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;

public class ALiLQR {
	Environment env;
	
	SimpleMatrix LinearFeedback; //K
	SimpleMatrix FeedForward; //d
 	double expdV;
	
	void backwardsPass(ArrayList<SimpleMatrix> X, ArrayList<SimpleMatrix> U) {
		SimpleMatrix terminalGradient;
		SimpleMatrix terminalHessian;
		for(int k = env.getHorizonLen() - 1; k > 0; k--) {
			SimpleMatrix dQ = new SimpleMatrix(env.getXDim(), 1);
			if(positiveDefinite(env.avHessianUU(X.get(k), U.get(k), k))) {
				SimpleMatrix term = env.avHessianUU(X.get(k), U.get(k), k).invert().negative();
				
				LinearFeedback = term.mult((env.avHessianXU(X.get(k), U.get(k), k)).transpose()); //K
				FeedForward = term.mult(env.avJacobianU(X.get(k), U.get(k), k)); //d
				expdV = (FeedForward.transpose().mult(env.avJacobianU(X.get(k), U.get(k), k)).plus(FeedForward.transpose().mult(env.avHessianUU(X.get(k), U.get(k), k).mult(FeedForward).scale(0.5))).get(0));
			}else {
				//increase p and reset
			}
		}
		
	}
	boolean positiveDefinite(SimpleMatrix toCheck) {
		boolean posDef = true;
		SimpleEVD<SimpleMatrix> eig = toCheck.eig();
		for(int i = 0; i < eig.getIndexMax() && posDef; i ++) if(eig.getEigenvalue(i).getReal() < 0) posDef = false;
		return posDef;
	}
	void forwardsPass(ArrayList<SimpleMatrix> X, ArrayList<SimpleMatrix> U) {
		SimpleMatrix nextState = X.get(0);
		double alpha = 1;
		double lastCost, cost;
		
	}
	void augmentedLagrangianIteratedLinearQuadraticRegulator() {
		
	}
	void lineSearch() {
		
	}
}
