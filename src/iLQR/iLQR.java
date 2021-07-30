package iLQR;

import java.util.ArrayList;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.qr.QRColPivDecompositionHouseholderColumn_DDRM;

import org.ejml.simple.SimpleMatrix;

public class iLQR {
	//Plagarism
	//https://github.com/vvanirudh/iLQR/blob/master/ilqr.h
	static final double DefaultStepSize = 0.0009765625;
	
	Function f;
	Function ct;
	Function cell;
	Function quadratizeCost;
	Function quadratizeFinalCost;
	
	SimpleMatrix dynJacobianX(SimpleMatrix a, SimpleMatrix b, double step){
		SimpleMatrix A = new SimpleMatrix(a.numRows(), b.numRows());
		SimpleMatrix ar = a;
		SimpleMatrix al = a;
		
		for(int i = 0; i < a.numRows(); i++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, al.get(i) - step);
			
			for(int j = 0; j < a.numRows(); j++) {
				A.combine(i, 0, (f.function(ar, b).minus(f.function(al, b))).divide(2 * step));
			}
			ar.set(i, 0, a.get(i, 0));
			al.set(i, 0, a.get(i, 0));
		}
		return A;
	}
	SimpleMatrix dynJacobianX(SimpleMatrix a, SimpleMatrix b){
		return dynJacobianX(a, b, DefaultStepSize);
	}
	SimpleMatrix dynJacobianU(SimpleMatrix a, SimpleMatrix b, double step){
		SimpleMatrix B = new SimpleMatrix(a.numRows(), b.numRows());
		SimpleMatrix br = b;
		SimpleMatrix bl = b;
		
		for(int i = 0; i < b.numRows(); i++) {
			br.set(i, br.get(i) + step);
			bl.set(i, bl.get(i) - step);
			
			for(int j = 0; j < b.numRows(); j++) {
				B.combine(i, 0, (f.function(br, b).minus(f.function(bl, b))).divide(2 * step));
			}
			br.set(i, 0, a.get(i, 0));
			bl.set(i, 0, a.get(i, 0));
		}
		return B;
	}
	SimpleMatrix dynJacobianU(SimpleMatrix a, SimpleMatrix b){
		return dynJacobianU(a, b, DefaultStepSize);
	}
	
	void iterativeLQR(int horizon, SimpleMatrix initState, SimpleMatrix uNominal, int maxIter) {
		ArrayList<SimpleMatrix> L = new ArrayList<>();
		ArrayList<SimpleMatrix> l = new ArrayList<>();
		for(int i = 0; i < horizon; i++) {
			L.add(new SimpleMatrix(initState.numRows(), uNominal.numRows()));
		}
		for(int i = 0; i < horizon; i++) {
			l.add(uNominal);
		}
		
		ArrayList<SimpleMatrix> xHat = new ArrayList<>();
		ArrayList<SimpleMatrix> xHatNew = new ArrayList<>();
		for(int i = 0; i < horizon + 1; i++) {
			xHat.add(new SimpleMatrix(initState.numRows(), 1));
		}
		for(int i = 0; i < horizon + 1; i++) {
			xHatNew.add(new SimpleMatrix(initState.numRows(), 1));
		}
		
		ArrayList<SimpleMatrix> uHat = new ArrayList<>();
		ArrayList<SimpleMatrix> uHatNew = new ArrayList<>();
		for(int i = 0; i < horizon; i++) {
			uHat.add(new SimpleMatrix(uNominal.numRows(), 1));
		}
		for(int i = 0; i < horizon; i++) {
			uHatNew.add(new SimpleMatrix(uNominal.numRows(), 1));
		}
		
		double oldCost = -Math.log(0);
		
		for(int iter = 0; iter < maxIter; iter++) {
			double newCost;
			double alpha = 0;
			
			//Forward Pass to get Nominal Trajectory
			do {
				newCost = 0;
				
				//Initialize Trajectory
				xHatNew.set(0, initState);
				for(int t = 0; t < horizon; t++) {
					//Compute Control
					uHatNew.set(t, uHat.get(t).scale(1 - alpha).plus(L.get(t).mult(xHatNew.get(t).minus(xHat.get(t).scale(1-alpha)))).plus(l.get(t).scale(t)));
					
					//Forward Step
					xHatNew.set(t+1, f.function(xHatNew.get(t), uHatNew.get(t)));
					
					//Compute Cost
					//newCost += ct.function(xHatNew.get(t), uHatNew.get(t), t); //Sadgers Need Gooder Java
				}
				
				//Compute final state cost
				//newCost += cell.function(xHatNew.get(horizon));
				
				//Decrease alpha if new cost isn't less than old cost
				alpha *= 0.5;
			} while(!(newCost < oldCost) || Math.abs((oldCost - newCost) / newCost) < 1e-4);
			
			xHat = xHatNew;
			uHat = uHatNew;
			
			if(Math.abs((oldCost - newCost) / newCost) < 1e-4) {
				return;
			}
			
			oldCost = newCost;
			
			//Backwards Pass to compute control updates
			SimpleMatrix S = new SimpleMatrix(1, 1);;
			SimpleMatrix s = new SimpleMatrix(1, 1);; //v
			
			//Compute Final Cost
			
			for(int t = horizon - 1; t!= -1; t--) {
				//Compute Jacobians of x and u
				final SimpleMatrix A = new SimpleMatrix(1, 1);//Jacobian WRT X
				final SimpleMatrix B = new SimpleMatrix(1, 1);//Jacobian WRT U
				final SimpleMatrix c = new SimpleMatrix(1, 1);//xHat.get(t + 1).minus(A.mult(xHat.get(t))).minus(B.mult(uHat.get(t)));
				
				SimpleMatrix P = new SimpleMatrix(1, 1);;
				SimpleMatrix Q = new SimpleMatrix(1, 1);;
				SimpleMatrix R = new SimpleMatrix(1, 1);;
				SimpleMatrix q = new SimpleMatrix(1, 1);;
				SimpleMatrix r = new SimpleMatrix(1, 1);;
				
				//Quadratize Cost
				
				final SimpleMatrix C = B.transpose().mult(S).mult(A).plus(P);
				final SimpleMatrix D = A.transpose().mult(S).mult(A).plus(Q);
				final SimpleMatrix E = B.transpose().mult(S).mult(B).plus(R);
				final SimpleMatrix d = A.transpose().mult(s.plus(S.mult(c))).plus(q);
				final SimpleMatrix e = B.transpose().mult(s.plus(S.mult(c))).plus(r);
				
				QRColPivDecompositionHouseholderColumn_DDRM QRDecomp = new QRColPivDecompositionHouseholderColumn_DDRM();
				DMatrixRMaj typesub = new DMatrixRMaj(E.getMatrix());
				
				QRDecomp.decompose(typesub);
				SimpleMatrix typeresub = new SimpleMatrix(QRDecomp.getQR());
				
				L.set(t, typeresub.solve(C).negative());
				l.set(t, typeresub.solve(e).negative());
				
				S = D.plus(C.transpose().mult(L.get(t)));
				s = d.plus(c.transpose().mult(l.get(t)));
				/*
				 	L[t] = -(E.colPivHouseholderQr().solve(C));
		            l[t] = -(E.colPivHouseholderQr().solve(e));
		
		            S = D + C.transpose()*L[t];
		            s = d + C.transpose()*l[t];
				 */
			}
		}
	}
}
