package iLQR;

import java.util.ArrayList;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.qr.QRColPivDecompositionHouseholderColumn_DDRM;
import org.ejml.simple.SimpleMatrix;

public class DiffDriveILQR {
	static final double DefaultStepSize = 0.0009765625; //1/1024 for binary accuracy :)
	
	SimpleMatrix initState;
	SimpleMatrix initControl;
	
	SimpleMatrix defaultTargetState;
	SimpleMatrix defaultControlState;
	
	SimpleMatrix costQ; //Scaling for State Error
	SimpleMatrix costR; //Scaling for Control Error
	double costq; //Scaling for distance from obstacles
	
	ArrayList<SimpleMatrix> obstacles;
	
	DiffDriveILQR(SimpleMatrix initState, SimpleMatrix initControl, SimpleMatrix targetState, SimpleMatrix controlState, ArrayList<SimpleMatrix> obstacles, SimpleMatrix Q, SimpleMatrix R, double q, ArrayList<SimpleMatrix> obstaclePositions){
		this.initState = initState;
		this.initControl = initControl;
		this.defaultTargetState = targetState;
		this.defaultControlState = controlState;
		this.obstacles = obstacles;
		this.costQ = Q;
		this.costR = R;
		this.costq = q;
	}
	
	//Diff Drive Dynamics
	SimpleMatrix Dynamics(SimpleMatrix State, SimpleMatrix Control) {
		//X = Column Vector [Px, Py, Theta]
		//U = Column Vector [Vel Left, Vel Right]
		SimpleMatrix XDot = new SimpleMatrix(State.numRows(), State.numCols());
		final double speed = (Control.get(0) + Control.get(1))/2;
		final double wheelDist = 1;
		XDot.set(0, speed * Math.cos(State.get(2)));
		XDot.set(1, speed * Math.sin(State.get(2)));
		XDot.set(2, (Control.get(1) - Control.get(0)/wheelDist));
		
		return XDot;
	}
	SimpleMatrix dynJacobianX(SimpleMatrix a, SimpleMatrix b, double step){
		SimpleMatrix A = new SimpleMatrix(3, a.numRows());
		SimpleMatrix ar = a;
		SimpleMatrix al = a;
		
		for(int i = 0; i < a.numRows(); i++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, al.get(i) - step);
			
			A.insertIntoThis(0, i, (Dynamics(ar, b).minus(Dynamics(al, b))).divide(2 * step));
			
			ar.set(i, 0, a.get(i, 0));
			al.set(i, 0, a.get(i, 0));
		}
		return A;
	}
	SimpleMatrix dynJacobianX(SimpleMatrix a, SimpleMatrix b){
		return dynJacobianX(a, b, DefaultStepSize);
	}
	SimpleMatrix dynJacobianU(SimpleMatrix a, SimpleMatrix b, double step){
		SimpleMatrix B = new SimpleMatrix(3, b.numRows());
		SimpleMatrix br = b;
		SimpleMatrix bl = b;
		
		for(int i = 0; i < b.numRows(); i++) {
			br.set(i, br.get(i) + step);
			bl.set(i, bl.get(i) - step);
			
			B.insertIntoThis(0, i, (Dynamics(a, br).minus(Dynamics(a, bl))).divide(2 * step));
			
			br.set(i, 0, a.get(i, 0));
			bl.set(i, 0, a.get(i, 0));
		}
		return B;
	}
	SimpleMatrix dynJacobianU(SimpleMatrix a, SimpleMatrix b){
		return dynJacobianU(a, b, DefaultStepSize);
	}
	SimpleMatrix cEllJacobian(SimpleMatrix a, double step){
		//Its just the derivative KEKW
		SimpleMatrix A = new SimpleMatrix(a.numRows(), 1);
		SimpleMatrix ar = a;
		SimpleMatrix al = a;
		
		for(int i = 0; i < a.numRows(); i++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, al.get(i) - step);
			
			A.set(i, 0, (cEll(ar) - (cEll(al)))/(2 * step));
			
			ar.set(i, 0, a.get(i, 0));
			al.set(i, 0, a.get(i, 0));
		}
		return A;
	}
	SimpleMatrix cEllJacobian(SimpleMatrix a) {
		return cEllJacobian(a, DefaultStepSize);
	}
	SimpleMatrix cEllHessian(SimpleMatrix State, double step) {
		SimpleMatrix ar = State;
		SimpleMatrix al = State;
		
		//double p = cEll(State);
		
		SimpleMatrix Q = new SimpleMatrix(State.numRows(), State.numRows());
		
		for(int i = 0; i < State.numRows(); i ++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, ar.get(i) - step);
			Q.set(i, i, (cEll(al) - 2 * cEll(State) + cEll(ar))/(step * step));
			ar.set(i, State.get(i));
			al.set(i, State.get(i));
		}
		
		SimpleMatrix atr = State;
		SimpleMatrix atl = State;
		SimpleMatrix abr = State;
		SimpleMatrix abl = State;
		
		for(int i = 0; i < State.numRows(); i ++) {
			atr.set(i, atr.get(i) + step);
			atl.set(i, atl.get(i) - step);
			abr.set(i, abr.get(i) + step);
			abl.set(i, abl.get(i) - step);
			for(int j = 0; j < i; j ++) {
				atr.set(i, atr.get(j) + step);
				atl.set(i, atl.get(j) - step);
				abr.set(i, abr.get(j) + step);
				abl.set(i, abl.get(j) - step);
				Q.set(i, j, (cEll(abl) + cEll(atr) - cEll(atl) - cEll(abr))/(4 * step * step));
				Q.set(j, i, Q.get(i, j));
				atr.set(j, State.get(j));
				atl.set(j, State.get(j));
				abr.set(j, State.get(j));
				abl.set(j, State.get(j));
			}
			atr.set(i, State.get(i));
			atl.set(i, State.get(i));
			abr.set(i, State.get(i));
			abl.set(i, State.get(i));
		}
		return Q;
	}
	SimpleMatrix cEllHessian(SimpleMatrix State) {
		return cEllHessian(State, DefaultStepSize);
	}
	SimpleMatrix cTJacobian1(SimpleMatrix State, SimpleMatrix Control, double step){
		SimpleMatrix A = new SimpleMatrix(1, State.numRows());
		SimpleMatrix ar = State;
		SimpleMatrix al = State;
		
		for(int i = 0; i < State.numRows(); i++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, al.get(i) - step);
			
			for(int j = 0; j < State.numRows(); j++) {
				A.set(0, j, (cT(ar, Control) - (cT(al, Control)))/(2 * step));
			}
			
			ar.set(i, 0, State.get(i, 0));
			al.set(i, 0, State.get(i, 0));
		}
		A.print();
		return A;
	}
	SimpleMatrix cTJacobian1(SimpleMatrix State, SimpleMatrix Control) {
		return cTJacobian1(State, Control, DefaultStepSize);
	}
	SimpleMatrix cTJacobian2(SimpleMatrix State, SimpleMatrix Control, double step){
		SimpleMatrix B = new SimpleMatrix(1, Control.numRows());
		SimpleMatrix br = Control;
		SimpleMatrix bl = Control;
		
		for(int i = 0; i < Control.numRows(); i++) {
			br.set(i, br.get(i) + step);
			bl.set(i, bl.get(i) - step);
			
			for(int j = 0; j < Control.numRows(); j++) {
				B.set(0, j, (cT(State, br) - (cT(State, bl)))/(2 * step));
			}
			br.set(i, 0, Control.get(i, 0));
			bl.set(i, 0, Control.get(i, 0));
		}
		B.print();
		return B;
	}
	SimpleMatrix cTJacobian2(SimpleMatrix State, SimpleMatrix Control) {
		return cTJacobian2(State, Control, DefaultStepSize);
	}
	SimpleMatrix cTHessian1(SimpleMatrix State, SimpleMatrix Control, double step) {
		SimpleMatrix ar = State;
		SimpleMatrix al = State;
		
		//double p = cT(State, Control);
		
		SimpleMatrix Q = new SimpleMatrix(State.numRows(), State.numRows());
		
		for(int i = 0; i < State.numRows(); i ++) {
			ar.set(i, ar.get(i) + step);
			al.set(i, ar.get(i) - step);
			Q.set(i, i, (cT(al, Control) - 2 * cT(State, Control) + cT(ar, Control))/(step * step));
			ar.set(i, State.get(i));
			al.set(i, State.get(i));
		}
		
		SimpleMatrix atr = State;
		SimpleMatrix atl = State;
		SimpleMatrix abr = State;
		SimpleMatrix abl = State;
		
		for(int i = 0; i < State.numRows(); i ++) {
			atr.set(i, atr.get(i) + step);
			atl.set(i, atl.get(i) - step);
			abr.set(i, abr.get(i) + step);
			abl.set(i, abl.get(i) - step);
			for(int j = 0; j < i; j ++) {
				atr.set(i, atr.get(j) + step);
				atl.set(i, atl.get(j) - step);
				abr.set(i, abr.get(j) + step);
				abl.set(i, abl.get(j) - step);
				Q.set(i, j, (cT(abl, Control) + cT(atr, Control) - cT(atl, Control) - cT(abr, Control))/(4 * step * step));
				Q.set(j, i, Q.get(i, j));
				atr.set(j, State.get(j));
				atl.set(j, State.get(j));
				abr.set(j, State.get(j));
				abl.set(j, State.get(j));
			}
			atr.set(i, State.get(i));
			atl.set(i, State.get(i));
			abr.set(i, State.get(i));
			abl.set(i, State.get(i));
		}
		return Q;
	}
	SimpleMatrix cTHessian1(SimpleMatrix State, SimpleMatrix Control) {
		return cTHessian1(State, Control, DefaultStepSize);
	}
	SimpleMatrix cTHessian2(SimpleMatrix State, SimpleMatrix Control, double step) {
		SimpleMatrix br = Control;
		SimpleMatrix bl = Control;
		
		//double p = cT(State, Control);
		
		SimpleMatrix Q = new SimpleMatrix(State.numRows(), State.numRows());
		
		for(int i = 0; i < Control.numRows(); i ++) {
			br.set(i, br.get(i) + step);
			bl.set(i, br.get(i) - step);
			Q.set(i, i, (cT(State, bl) - 2 * cT(State, Control) + cT(State, br))/(step * step));
			br.set(i, State.get(i));
			bl.set(i, State.get(i));
		}
		
		SimpleMatrix btr = Control;
		SimpleMatrix btl = Control;
		SimpleMatrix bbr = Control;
		SimpleMatrix bbl = Control;
		
		for(int i = 0; i < Control.numRows(); i ++) {
			btr.set(i, btr.get(i) + step);
			btl.set(i, btl.get(i) - step);	
			bbr.set(i, bbr.get(i) + step);
			bbl.set(i, bbl.get(i) - step);
			for(int j = 0; j < i; j ++) {
				btr.set(i, btr.get(j) + step);
				btl.set(i, btl.get(j) - step);
				bbr.set(i, bbr.get(j) + step);
				bbl.set(i, bbl.get(j) - step);
				Q.set(i, j, (cT(State, bbl) + cT(State, btr) - cT(State, btl) - cT(State, bbr))/(4 * step * step));
				Q.set(j, i, Q.get(i, j));
				btr.set(j, State.get(j));
				btl.set(j, State.get(j));
				bbr.set(j, State.get(j));
				bbl.set(j, State.get(j));
			}
			btr.set(i, State.get(i));
			btl.set(i, State.get(i));
			bbr.set(i, State.get(i));
			bbl.set(i, State.get(i));
		}
		return Q;
	}
	SimpleMatrix cTHessian2(SimpleMatrix State, SimpleMatrix Control) {
		return cTHessian2(State, Control, DefaultStepSize);
	}
	SimpleMatrix cTHessian12(SimpleMatrix State, SimpleMatrix Control, double step) {
		SimpleMatrix Q = new SimpleMatrix(State.numRows(), Control.numRows());
		
		SimpleMatrix atr = State;
		SimpleMatrix atl = State;
		SimpleMatrix abr = State;
		SimpleMatrix abl = State;
		SimpleMatrix btr = Control;
		SimpleMatrix btl = Control;
		SimpleMatrix bbr = Control;
		SimpleMatrix bbl = Control;
		
		for(int i = 0; i < State.numRows(); i ++) {
			atr.set(i, atr.get(i) + step);
			atl.set(i, atl.get(i) - step);
			abr.set(i, abr.get(i) + step);
			abl.set(i, abl.get(i) - step);
			for(int j = 0; j < Control.numRows(); j ++) {
				btr.set(j, atr.get(j) + step);
				btl.set(j, atl.get(j) - step);
				bbr.set(j, abr.get(j) + step);
				bbl.set(j, abl.get(j) - step);
				Q.set(i, j, (cT(abl, bbl) + cT(atr, btr) - cT(atl, btl) - cT(abr, bbr))/(4 * step * step));
				btr.set(j, State.get(j));
				btl.set(j, State.get(j));
				bbr.set(j, State.get(j));
				bbl.set(j, State.get(j));
			}
			atr.set(i, State.get(i));
			atl.set(i, State.get(i));
			abr.set(i, State.get(i));
			abl.set(i, State.get(i));
		}
		return Q;
	}
	SimpleMatrix cTHessian12(SimpleMatrix State, SimpleMatrix Control) {
		return cTHessian12(State, Control, DefaultStepSize);
	}
	double cZero(SimpleMatrix State, SimpleMatrix targetState, SimpleMatrix Q) {
		return ((State.minus(targetState))).transpose().mult(Q).mult(State.minus(targetState)).get(0);
	}
	double cZero(SimpleMatrix State) {
		return cZero(State, defaultTargetState, costQ);
	}
	double cEll(SimpleMatrix State, SimpleMatrix initState, SimpleMatrix Q) {
		return ((State.minus(initState))).transpose().mult(Q).mult(State.minus(initState)).get(0);
	}
	double cEll(SimpleMatrix State) {
		return cEll(State, initState, costQ);
	}
	double ObstacleCost(ArrayList<SimpleMatrix> ObjectPositions, SimpleMatrix State) {
		double sum = 0;
		for(int i = 0; i < ObjectPositions.size(); i++) {
			double dist = Math.hypot(ObjectPositions.get(i).get(0) - State.get(0), ObjectPositions.get(i).get(1) - State.get(1));
			sum += Math.exp(-dist);
		}
		//Regularize this term (XD)
		if(sum < 0) return 0;
		return sum;
	}
	double ObstacleCost(SimpleMatrix State) {
		return ObstacleCost(obstacles, State);
	}
	double cT(SimpleMatrix State, SimpleMatrix Control, SimpleMatrix targetControl, SimpleMatrix R, double q) {
		return ((Control.minus(targetControl))).transpose().mult(R).mult(Control.minus(targetControl)).get(0) + ObstacleCost(State);
	}
	double cT(SimpleMatrix State, SimpleMatrix Control) {
		return cT(State, Control, defaultControlState, costR, costq);
	}
	
	void iterativeLQR(int horizon, SimpleMatrix initState, SimpleMatrix uNominal, int maxIter) {
		ArrayList<SimpleMatrix> L = new ArrayList<>();
		ArrayList<SimpleMatrix> l = new ArrayList<>();
		for(int i = 0; i < horizon; i++) {
			L.add(new SimpleMatrix(uNominal.numRows(), initState.numRows()));
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
			double alpha = 1;
			
			//Forward Pass to get Nominal Trajectory
			do {
				newCost = 0;
				
				//Initialize Trajectory
				xHatNew.set(0, initState);
				for(int t = 0; t < horizon; t++) {
					//Compute Control
					//uHatNew[t] = (1.0 - alpha)*uHat[t] + L[t]*(xHatNew[t] - (1.0 - alpha)*xHat[t]) + alpha*l[t];
					SimpleMatrix Term1 = uHat.get(t).scale(1 - alpha);
					SimpleMatrix Term3 = xHatNew.get(t).minus(xHat.get(t).scale(1-alpha));
					SimpleMatrix Term4 = l.get(t).scale(alpha);
					
//					xHatNew.get(t).print();
//					xHat.get(t).scale(1-alpha).print();
//					xHatNew.get(t).minus(xHat.get(t).scale(1-alpha)).print();
//					Term3.print();
//					L.get(t).print();
//					Term3.print();
//					L.get(t).mult(Term3).print();
//					Term1.plus(L.get(t).mult(Term3)).print();
//					Term1.plus(L.get(t).mult(Term3)).plus(Term4).print();
					
					uHatNew.set(t, Term1.plus(L.get(t).mult(Term3)).plus(Term4));
					
					//Forward Step
					xHatNew.set(t+1, Dynamics(xHatNew.get(t), uHatNew.get(t)));
					
					//Compute Cost
					newCost += cT(xHatNew.get(t), uHatNew.get(t));
				}
				
				//Compute final state cost
				newCost += cEll(xHatNew.get(horizon));
				
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
			SimpleMatrix S = cEllHessian(xHat.get(horizon));
			SimpleMatrix s = cEllJacobian(xHat.get(horizon)).minus(S.mult(xHat.get(horizon))); //v
			
			//Compute Final Cost
			
			for(int t = horizon - 1; t!= -1; t--) {
				//Compute Jacobians of x and u, to quadratize cost
				final SimpleMatrix A = dynJacobianX(xHat.get(t), uHat.get(t));//Jacobian WRT X
				final SimpleMatrix B = dynJacobianU(xHat.get(t), uHat.get(t));//Jacobian WRT U
				
				
//				xHat.get(t).print();
//				A.print();
//				B.print();
//				B.mult(uHat.get(t)).print();
//				A.mult(xHat.get(t)).print();
				
				final SimpleMatrix c = xHat.get(t + 1).minus(A.mult(xHat.get(t))).minus(B.mult(uHat.get(t)));
				//xHat.get(t + 1) - (A.mult(xHat.get(t))) - (B.mult(uHat.get(t)));
				
				SimpleMatrix P = cTHessian12(xHat.get(t), uHat.get(t));
				SimpleMatrix Q = cTHessian1(xHat.get(t), uHat.get(t));
				SimpleMatrix R = cTHessian2(xHat.get(t), uHat.get(t));
				//cT
				SimpleMatrix q = cTJacobian1(xHat.get(t), uHat.get(t)).minus(Q.mult(xHat.get(t))).minus(P.transpose().mult(uHat.get(t)));
				SimpleMatrix r = cTJacobian2(xHat.get(t), uHat.get(t)).minus(P.mult(xHat.get(t))).minus(R.mult(uHat.get(t)));
				
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
			}
		}
	}
}
