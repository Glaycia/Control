package LQR;
import org.ejml.simple.*;
public class LQRFactory {
	
	//Poles are placed on the Pareto Boundary
	//Poles should always be placed on the Left Half Plane
	
	//http://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf
	//Section 2 tells how to get such Q1, Q2...
	//TLDR Choose manually what kind of state and control costs are ok, then tune Rho by guessing. Greater Values of Rho penalize Control Effort while lesser penalize state excursions
	
	SimpleMatrix A; //States x States
	SimpleMatrix B; //States x Controls
	SimpleMatrix Q; //States x States
	SimpleMatrix R; //Controls x Controls
	//SimpleMatrix U; //Controls x 1
	SimpleMatrix S; //Solving Matrix
	public SimpleMatrix K; //Controller Gain Matrix
	
	
	public LQRFactory() {}
	public LQRFactory(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
		this.A = A;
		this.B = B;
		this.Q = Q;
		this.R = R;
	}
	void CARERicattiArimotoPotter() {
		//This one is Continuous, larger gains
		//https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp
		//https://github.com/YunyiShen/AREsolver/blob/master/src/Riccati_Solver.cpp
		int dim_X = A.numRows();
		
		//Hamilton Matrix
		SimpleMatrix Ham = new SimpleMatrix(2 * dim_X, 2 * dim_X);
		Ham.combine(0, 0, A);
		Ham.combine(0, dim_X, (B).negative().mult(R.invert()).mult(B.transpose()));
		Ham.combine(dim_X, 0, Q.negative());
		Ham.combine(dim_X, dim_X, A.transpose().negative());
		
		SimpleEVD<SimpleMatrix> Eigens = Ham.eig();
		SimpleMatrix EigenVector = new SimpleMatrix(2 * dim_X, dim_X);
		int j = 0;
		for(int i = 0; i < 2 * dim_X; i++) {
			if(Eigens.getEigenvalue(i).getReal() < 0) {
				EigenVector.combine(j, 0, Eigens.getEigenVector(i));
				j++;
			}
		}
		SimpleMatrix M1 = EigenVector.cols(0, dim_X - 1);
		SimpleMatrix M2 = EigenVector.cols(dim_X - 1, dim_X * 2 - 1);
		
		S = M1.invert().mult(M2);
	}
	void DARERicattiArimotoPotter() {
		//This one is Discrete, smaller gains to account for sampling
		//https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp
		//https://github.com/YunyiShen/AREsolver/blob/master/src/Riccati_Solver.cpp
		int dim_X = A.numRows();
		
		SimpleMatrix BiRBt = B.mult(R.solve(B.transpose()));
		
		//Hamilton Matrix
		SimpleMatrix Ham = new SimpleMatrix(2 * dim_X, 2 * dim_X);
		Ham.combine(0, 0, A.plus(BiRBt.mult(A.invert().transpose().mult(Q))));
		Ham.combine(0, dim_X, BiRBt.mult(A.invert().transpose()).negative());
		Ham.combine(dim_X, 0, A.invert().transpose().mult(Q).negative());
		Ham.combine(dim_X, dim_X, (A.invert().transpose()));
		
		SimpleEVD<SimpleMatrix> Eigens = Ham.eig();
		SimpleMatrix EigenVector = new SimpleMatrix(2 * dim_X, dim_X);
		int j = 0;
		for(int i = 0; i < 2 * dim_X; i++) {
			if(Eigens.getEigenvalue(i).getReal() < 0) {
				EigenVector.combine(j, 0, Eigens.getEigenVector(i));
				j++;
			}
		}

		SimpleMatrix M1 = EigenVector.cols(0, dim_X - 1);
		SimpleMatrix M2 = EigenVector.cols(dim_X - 1, dim_X * 2 - 1);
		
		S = M1.invert().mult(M2);
	}
	public void DAREIteration(int maxIter, double tolerance) {
		//https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp
		SimpleMatrix P = Q;
		
		SimpleMatrix PNext;
		
		SimpleMatrix AT = A.transpose();
		SimpleMatrix BT = B.transpose();
		//SimpleMatrix RInv = R.invert();
		
		double diff;
		
		for(int i = 0; i < maxIter; i++) {
			//BT.mult(P).mult(B).invert().print();
			
			SimpleMatrix Term1 = AT.mult(P).mult(A);
			SimpleMatrix Term2 = R.plus(BT.mult(P).mult(B)).invert();
			SimpleMatrix Term3 = AT.mult(P).mult(B).mult(Term2);
			SimpleMatrix Term4 = (BT).mult(P).mult(A);
			
			PNext = Term1.minus((Term3).mult(Term4)).plus(Q);
			diff = (PNext.minus(P).elementMaxAbs());
			P = PNext;
			if(diff < tolerance) {
				S = P;
			}
		}
		
	}
	public void computeK() {
		K = R.invert().mult(B.transpose()).mult(S);
	}
}
