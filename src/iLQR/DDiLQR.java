package iLQR;

import java.util.ArrayList;

import org.ejml.data.Matrix;
import org.ejml.simple.SimpleBase;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

public class DDiLQR {
	Environment env;
	
	ArrayList<SimpleMatrix> X = new ArrayList<>();
	ArrayList<SimpleMatrix> U = new ArrayList<>();
	ArrayList<SimpleMatrix> nX = new ArrayList<>();
	ArrayList<SimpleMatrix> nU = new ArrayList<>();
	
	public DDiLQR(SimpleMatrix startState, SimpleMatrix nominalControl){
		for(int i = 0; i <= env.getHorizonLen(); i++) {
			X.add(new SimpleMatrix(startState));
		}
		for(int i = 0; i < env.getHorizonLen(); i ++) {
			U.add(new SimpleMatrix(nominalControl));
		}
		for(int i = 0; i <= env.getHorizonLen(); i++) {
			nX.add(new SimpleMatrix(startState));
		}
		for(int i = 0; i < env.getHorizonLen(); i ++) {
			nU.add(new SimpleMatrix(nominalControl));
		}
	}
	void backwardsPass(ArrayList<SimpleMatrix> X, ArrayList<SimpleMatrix> U) {
		
	}
	void forwardsPass(ArrayList<SimpleMatrix> X, ArrayList<SimpleMatrix> U) {
		nX.set(0, X.get(0));
		for(int i; i < env.getHorizonLen(); i++) {
			SimpleSVD A = new SimpleSVD((Matrix) env.avHessianUU(X.get(i), U.get(i), i), false);
			SimpleBase aU = A.getU();
			SimpleBase aV = A.getV();
			SimpleBase aW = A.getW();
			for(int j = 0; i < aW.numRows() * aW.numCols(); i++) {
				aW.set(i, 1/aW.get(i));
				if(aW.get(i) < 0)aW.set(i, 0);
			}
;;;;;;;;			
			SimpleMatrix QuuInverse = aU.dot(aW).dot(aV.transpose()); //Change
			
			SimpleMatrix k = QuuInverse.mult(env.avJacobianU(X.get(i), U.get(i), i)).negative();
			SimpleMatrix K = QuuInverse.mult(env.avHessianXU(X.get(i), U.get(i), i).transpose()).negative();
			nU.set(i, U.get(i).plus(k).plus(K.mult(nX.get(i).minus(X.get(i))))); //unt = ut + k + K (xnt - xt)
			nX.set(i + 1, env.dynamics(X.get(i), U.get(i)));
		}
		
	}
	void augmentedLagrangianIteratedLinearQuadraticRegulator() {
		
	}
	void lineSearch() {
		
	}
}
