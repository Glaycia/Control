import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
	//
	SimpleMatrix KalmanGain;
	
	SimpleMatrix State; //State Matrix
	SimpleMatrix PState; //Previous State Matrix
	
	SimpleMatrix YState; //Measurement of State
	
	SimpleMatrix P; //Process Covariance Matrix
	SimpleMatrix U; //Control Variable Matrix (System Model)
	SimpleMatrix Q; //Process Noise Covariance Matrix
	SimpleMatrix W; //Predicted State Noise Matrix
	SimpleMatrix Z; //Measurement Uncertainty Matrix
	SimpleMatrix H; //H is a conversion matrix in order to make the sizes the same so you can multiply the matrices.
	SimpleMatrix R; //Sensor Noise Covariance Matrix
	
	SimpleMatrix A; //Adaptation Matrix for Previous State
	SimpleMatrix B; //Adaptation Matrix for Control Variables
	SimpleMatrix C; //Adaptation Matrix for Measurement State
	
	
	
	void Update(SimpleMatrix Measurement) {
		State = A.mult(PState).plus(B.mult(U)).plus(W);
		P = A.mult(PState).mult(A.transpose()).plus(Q);
		PState = State;
		
		YState = C.mult(Measurement).plus(Z);
		
		KalmanGain = P.mult(H.transpose()).mult((H.mult(P).mult(H.transpose()).plus(R)).transpose());
		
		State = PState.plus(KalmanGain.mult(YState.minus(H.mult(PState))));
		PState = State;
		
		P = (SimpleMatrix.identity(KalmanGain.numRows()).minus(KalmanGain.mult(H)));
	}
	
	SimpleMatrix Covariance(SimpleMatrix Data) {
		//Data comes in with each column as an attribute
		SimpleMatrix Covariances = new SimpleMatrix(Data.numCols(), Data.numCols());
		SimpleMatrix ConstMatrix = new SimpleMatrix(Data.numRows(), Data.numRows());
		SimpleMatrix Variance = new SimpleMatrix(Data.numRows(), Data.numCols());
		ConstMatrix.plus(1/((double)Data.numRows()));
		Variance = Data.minus(ConstMatrix.mult(Data));
		Covariances = Variance.transpose().mult(Variance);
		
		return Covariances;
	}
	
	
	
	
	
	
	
	
	
	
	
//	SimpleMatrix KalmanGainCalc(SimpleMatrix ErrorEst, SimpleMatrix ErrorMea) {
//		return ErrorEst.mult((ErrorEst.plus(ErrorMea)).invert());
//	}
//	SimpleMatrix Estimate(SimpleMatrix Measurement) {
//		//KalmanGain = KalmanGainCalc(CurrentStateError, CurrentMeaError);
//		
//		CurrentStateEst = PreviousStateEst.plus((Measurement.minus(PreviousStateEst).mult(KalmanGain)));
//		PreviousStateEst = CurrentStateEst;
//		
//		CurrentStateError = KalmanGain.negative().plus(1).mult(PreviousStateError);
//		PreviousStateError = CurrentStateError;
//		
//		return CurrentStateEst;
//	}
}
