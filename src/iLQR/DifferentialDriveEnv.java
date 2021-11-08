package iLQR;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import LQR.LQRFactory;

public class DifferentialDriveEnv implements Environment{
	int xDim = 3;
	int uDim = 2;
	
	static final double DefaultStepSize = 0.0009765625;
	
	int horizon = 100;
	double dT = 1/10;
	double robotRadius = 2;
	double wheelDist = 1/Math.PI; //Change, is hypoth rn
	
	ArrayList<SimpleMatrix> obsArray;
	
	//Model Matrix
	SimpleMatrix A = new SimpleMatrix(xDim, xDim);
	SimpleMatrix B = new SimpleMatrix(xDim, uDim);
	
	//Cost Factors
	double obstacleScaleFactor;
	SimpleMatrix stateCost;
	SimpleMatrix controlCost;
	
	//Trajectory Data
	ArrayList<SimpleMatrix> X;
	ArrayList<SimpleMatrix> L;
	ArrayList<SimpleMatrix> U;
	
	LQRFactory ricattiSolver = new LQRFactory(A, B, stateCost, controlCost);
	
	public DifferentialDriveEnv() {	
		for(int i = 0; i < horizon - 1; i ++) L.add(new SimpleMatrix(uDim, xDim));
		for(int i = 0; i < horizon - 1; i ++) U.add(new SimpleMatrix(uDim, 1));
		for(int i = 0; i < horizon; i ++) X.add(new SimpleMatrix(xDim, 1));
		
		A = SimpleMatrix.identity(3);
		B = B(0);
	}
	public int getXDim() {
		return xDim;
	}
	public int getUDim() {
		return uDim;
	}
	public int getHorizonLen() {
		return horizon;
	}
	SimpleMatrix B(double angle) {
		SimpleMatrix toReturn = new SimpleMatrix(xDim, uDim);
		toReturn.set(0, 0, Math.cos(angle) * dT);
		toReturn.set(1, 0, Math.sin(angle) * dT);
		toReturn.set(2, 1, dT);
		return toReturn;
	}
	public SimpleMatrix obstacle(double x, double y, double radius, double weight) {
		SimpleMatrix toReturn = new SimpleMatrix(4, 1);
		toReturn.set(0, x);
		toReturn.set(1, y);
		toReturn.set(2, radius);
		toReturn.set(3, weight);
		return toReturn;
	}
	public void initObstacles() {
		obsArray.add(obstacle(6, 9, 1, 1));
		obsArray.add(obstacle(20, 3, 1, 1));
	}
	public double obstacleCost(SimpleMatrix state) {
		double cost = 0;
		for(int i = 0; i < obsArray.size(); i++) cost += obsArray.get(i).get(3) * Math.exp(-(Math.hypot(state.get(0) - obsArray.get(i).get(0), state.get(1) - obsArray.get(i).get(1) - robotRadius - obsArray.get(i).get(2))));
		return cost * obstacleScaleFactor;
	}
	public double costToGo(SimpleMatrix state, SimpleMatrix control, int horizonStep) {
		SimpleMatrix stateTerm = state.transpose().mult(stateCost).mult(state);
		SimpleMatrix controlTerm = control.transpose().mult(controlCost).mult(control);
		ricattiSolver.DAREIteration(100, 0.001);
		ricattiSolver.computeK();
		SimpleMatrix P = ricattiSolver.K; //Ricatti P;
		SimpleMatrix crossTerm = (A.mult(state).plus(B.mult(control))).transpose().mult(P).mult(A.mult(state).plus(B.mult(control)));
		return (stateTerm.plus(controlTerm).plus(crossTerm)).scale(0.5).get(0);
		//return 4;
	}
	public double actionCost(SimpleMatrix state, SimpleMatrix control, int horizonStep) {
		if(horizonStep == horizon) return state.transpose().mult(stateCost).mult(state).scale(0.5).get(0);
		dynamics(state, control);
		ricattiSolver.DAREIteration(100, 0.001);
		ricattiSolver.computeK();
		SimpleMatrix P = ricattiSolver.K; //Ricatti P;
		return state.transpose().mult(P).mult(state).scale(0.5).get(0);
		//return 4;
	}
	public SimpleMatrix continuous(SimpleMatrix state, SimpleMatrix control) { //returns column vector, xDot;
		SimpleMatrix returnVector = new SimpleMatrix(3, 1);
		
		returnVector.set(0, (control.get(0) + control.get(1)) * (Math.cos(state.get(2)/2)));
		returnVector.set(1, (control.get(0) + control.get(1)) * (Math.sin(state.get(2)/2)));
		returnVector.set(2, (control.get(1) + control.get(0))/wheelDist);
		
		return returnVector;
	}
	public SimpleMatrix dynamics(SimpleMatrix state, SimpleMatrix control) { //returns column vector, x+1
		SimpleMatrix k1 = continuous(state, control);
		SimpleMatrix k2 = continuous(state.plus(k1.scale(dT/2)), control);
		SimpleMatrix k3 = continuous(state.plus(k2.scale(dT/2)), control);
		SimpleMatrix k4 = continuous(state.plus(k3.scale(dT)), control);
		
		SimpleMatrix toReturn = state.plus((k1.plus(k2.scale(2)).plus(k3.scale(2).plus(k4))).scale(dT/6));
		B = B(toReturn.get(3));
		return toReturn;
	}

	@Override
	public SimpleMatrix avHessianXX(SimpleMatrix x, SimpleMatrix u, int iter) {
		SimpleMatrix ddXX = new SimpleMatrix(xDim, xDim);
		double init = costToGo(x, u, iter);
		
		SimpleMatrix xr = x;
		SimpleMatrix xl = x;
		for(int i = 0; i < xDim; i ++) {
			xr.set(i, xr.get(i) + DefaultStepSize);
			xl.set(i, xl.get(i) - DefaultStepSize);
			ddXX.set(i, i, (costToGo(xl, u, iter) - 2 * init + costToGo(xr, u, iter))/(DefaultStepSize * DefaultStepSize));
			xr.set(i, x.get(i));
			xl.set(i, x.get(i));
		}
		SimpleMatrix xtr = x;
		SimpleMatrix xtl = x;
		SimpleMatrix xbr = x;
		SimpleMatrix xbl = x;
		for(int i = 1; i < xDim; i++) {
			xtr.set(i, xtr.get(i) + DefaultStepSize);
			xtl.set(i, xtl.get(i) - DefaultStepSize);
			xbr.set(i, xbr.get(i) + DefaultStepSize);
			xbl.set(i, xbl.get(i) - DefaultStepSize);
			for(int j = 0; j < i; j++) {
				xtr.set(j, xtr.get(j) + DefaultStepSize);
				xtl.set(j, xtl.get(j) + DefaultStepSize);
				xbr.set(j, xbr.get(j) - DefaultStepSize);
				xbl.set(j, xbl.get(j) - DefaultStepSize);
				ddXX.set(i, j, (costToGo(xtr, u, iter) + costToGo(xbl, u, iter) - costToGo(xtl, u, iter) - costToGo(xbr, u, iter))/(4 * DefaultStepSize * DefaultStepSize));
				xtr.set(j, x.get(j));
				xtl.set(j, x.get(j));
				xbr.set(j, x.get(j));
				xbl.set(j, x.get(j));
			}
			xtr.set(i, x.get(i));
			xtl.set(i, x.get(i));
			xbr.set(i, x.get(i));
			xbl.set(i, x.get(i));
		}
		return ddXX;
	}
	@Override
	public SimpleMatrix avHessianXU(SimpleMatrix x, SimpleMatrix u, int iter) {
		SimpleMatrix ddXU = new SimpleMatrix(xDim, uDim);

		SimpleMatrix xtr = x;
		SimpleMatrix xtl = x;
		SimpleMatrix xbr = x;
		SimpleMatrix xbl = x;
		SimpleMatrix utr = u;
		SimpleMatrix utl = u;
		SimpleMatrix ubr = u;
		SimpleMatrix ubl = u;
		for(int i = 0; i < xDim; i++) {
			xtr.set(i, xtr.get(i) + DefaultStepSize);
			xtl.set(i, xtl.get(i) - DefaultStepSize);
			xbr.set(i, xbr.get(i) + DefaultStepSize);
			xbl.set(i, xbl.get(i) - DefaultStepSize);
			for(int j = 0; j < uDim; j++) {
				utr.set(j, utr.get(j) + DefaultStepSize);
				utl.set(j, utl.get(j) + DefaultStepSize);
				ubr.set(j, ubr.get(j) - DefaultStepSize);
				ubl.set(j, ubl.get(j) - DefaultStepSize);
				ddXU.set(i, j, (costToGo(xtr, utr, iter) + costToGo(xbl, ubl, iter) - costToGo(xtl, utl, iter) - costToGo(xbr, ubr, iter))/(4 * DefaultStepSize * DefaultStepSize));
				utr.set(j, u.get(j));
				utl.set(j, u.get(j));
				ubr.set(j, u.get(j));
				ubl.set(j, u.get(j));
			}
			xtr.set(i, x.get(i));
			xtl.set(i, x.get(i));
			xbr.set(i, x.get(i));
			xbl.set(i, x.get(i));
		}
		return ddXU;
	}
	@Override
	public SimpleMatrix avHessianUU(SimpleMatrix x, SimpleMatrix u, int iter) {
		SimpleMatrix ddUU = new SimpleMatrix(xDim, xDim);
		double init = costToGo(x, u, iter);
		
		SimpleMatrix ur = u;
		SimpleMatrix ul = u;
		for(int i = 0; i < uDim; i ++) {
			ur.set(i, ur.get(i) + DefaultStepSize);
			ul.set(i, ul.get(i) - DefaultStepSize);
			ddUU.set(i, i, (costToGo(x, ul, iter) - 2 * init + costToGo(x, ur, iter))/(DefaultStepSize * DefaultStepSize));
			ur.set(i, u.get(i));
			ul.set(i, u.get(i));
		}
		SimpleMatrix utr = u;
		SimpleMatrix utl = u;
		SimpleMatrix ubr = u;
		SimpleMatrix ubl = u;
		for(int i = 1; i < uDim; i++) {
			utr.set(i, utr.get(i) + DefaultStepSize);
			utl.set(i, utl.get(i) - DefaultStepSize);
			ubr.set(i, ubr.get(i) + DefaultStepSize);
			ubl.set(i, ubl.get(i) - DefaultStepSize);
			for(int j = 0; j < i; j++) {
				utr.set(j, utr.get(j) + DefaultStepSize);
				utl.set(j, utl.get(j) + DefaultStepSize);
				ubr.set(j, ubr.get(j) - DefaultStepSize);
				ubl.set(j, ubl.get(j) - DefaultStepSize);
				ddUU.set(i, j, (costToGo(x, utr, iter) + costToGo(x, ubl, iter) - costToGo(x, utl, iter) - costToGo(x, ubr, iter))/(4 * DefaultStepSize * DefaultStepSize));
				utr.set(j, u.get(j));
				utl.set(j, u.get(j));
				ubr.set(j, u.get(j));
				ubl.set(j, u.get(j));
			}
			utr.set(i, u.get(i));
			utl.set(i, u.get(i));
			ubr.set(i, u.get(i));
			ubl.set(i, u.get(i));
		}
		return ddUU;
	}
	@Override
	public SimpleMatrix avJacobianX(SimpleMatrix x, SimpleMatrix u, int iter) {
		SimpleMatrix dX = new SimpleMatrix(xDim, 1);
		SimpleMatrix xr = x;
		SimpleMatrix xl = x;
		
		for(int i = 0; i < uDim; i++) {
			xr.set(i, xr.get(i) + DefaultStepSize);
			xl.set(i, xl.get(i) - DefaultStepSize);
			
			dX.set(i, 0, ((costToGo(xr, u, iter) - (costToGo(xl, u, iter))))/(2 * DefaultStepSize));
			
			xr.set(i, 0, x.get(i, 0));
			xl.set(i, 0, x.get(i, 0));
		}
		return dX;
	}
	@Override
	public SimpleMatrix avJacobianU(SimpleMatrix x, SimpleMatrix u, int iter) {
		SimpleMatrix dU = new SimpleMatrix(uDim, 1);
		SimpleMatrix ur = u;
		SimpleMatrix ul = u;
		
		for(int i = 0; i < uDim; i++) {
			ur.set(i, ur.get(i) + DefaultStepSize);
			ul.set(i, ul.get(i) - DefaultStepSize);
			
			dU.set(i, 0, ((costToGo(x, ur, iter) - (costToGo(x, ul, iter))))/(2 * DefaultStepSize));
			
			ur.set(i, 0, u.get(i, 0));
			ul.set(i, 0, u.get(i, 0));
		}
		return dU;
	}
	@Override
	public SimpleMatrix dynJacobianX(SimpleMatrix x, SimpleMatrix u) {
		SimpleMatrix dX = new SimpleMatrix(xDim, uDim);
		SimpleMatrix xr = x;
		SimpleMatrix xl = x;
		
		for(int i = 0; i < xDim; i++) {
			xr.set(i, xr.get(i) + DefaultStepSize);
			xl.set(i, xl.get(i) - DefaultStepSize);
			
			dX.insertIntoThis(i, 0, (dynamics(xr, u).minus(dynamics(xl, u))).divide(2 * DefaultStepSize));
			
			xr.set(i, 0, x.get(i, 0));
			xl.set(i, 0, x.get(i, 0));
		}
		return dX;
	}
	@Override
	public SimpleMatrix dynJacobianU(SimpleMatrix x, SimpleMatrix u) {
		SimpleMatrix dU = new SimpleMatrix(xDim, uDim);
		SimpleMatrix ur = u;
		SimpleMatrix ul = u;
		
		for(int i = 0; i < uDim; i++) {
			ur.set(i, ur.get(i) + DefaultStepSize);
			ul.set(i, ul.get(i) - DefaultStepSize);
			
			dU.insertIntoThis(i, 0, (dynamics(x, ur).minus(dynamics(x, ul))).divide(2 * DefaultStepSize));
			
			ur.set(i, 0, u.get(i, 0));
			ul.set(i, 0, u.get(i, 0));
		}
		return dU;
	}
}
