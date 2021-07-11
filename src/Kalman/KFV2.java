package Kalman;
import org.ejml.simple.SimpleMatrix;

public class KFV2 {
	//http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
	//https://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf
	SimpleMatrix State, Control, Covariance;
	SimpleMatrix A, B, C; //A: State Change Matrix, B: Input Effect Matrix, C: Output Adaptation Matrix
	SimpleMatrix Q, R; //Q: Process Noise Covariance Matrix, R: Measurement Covariance Matrix
	
	SimpleMatrix KalmanGain;
	SimpleMatrix PredictiveMeasurement;
	SimpleMatrix CovarianceMeasurement;
	
	int dataPointsS = 0;
	int CovarianceHorizonS = 10;
	SimpleMatrix PStateDataHolder;
	SimpleMatrix StateDataHolder;
	
	int dataPointsM = 0;
	int CovarianceHorizonM = 10;
	SimpleMatrix PMeasureDataHolder;
	SimpleMatrix MeasureDataHolder;
	
	public KFV2(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix State, SimpleMatrix Control, SimpleMatrix Covariance) {
		this.A = A;
		this.B = B;
		this.C = C;
		this.State = State;
		this.Control = Control;
		this.Covariance = Covariance;
	}
	public KFV2(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix State, SimpleMatrix Control) {
		this.A = A;
		this.B = B;
		this.C = C;
		this.State = State;
		this.Control = Control;
		this.Covariance = new SimpleMatrix(State.numRows(), State.numCols());
	}
	void addStateData() {
		if(dataPointsS != CovarianceHorizonS) dataPointsS++;
		
		
		StateDataHolder = new SimpleMatrix(dataPointsS, State.numRows());
		for(int i = 0; i < PStateDataHolder.numRows() - 1; i++) {
			for(int j = 0; j <PStateDataHolder.numCols(); i++) {
				StateDataHolder.set(i + 1, j, PStateDataHolder.get(i, j));
			}
		}
		
		for(int i = 0; i < State.numRows(); i++) StateDataHolder.set(0, i, State.get(i));
		
		PStateDataHolder = StateDataHolder;
	}
	void addMeasureData(SimpleMatrix measurement) {
		if(dataPointsM != CovarianceHorizonM) dataPointsM++;
		
		
		MeasureDataHolder = new SimpleMatrix(dataPointsM, measurement.numRows());
		for(int i = 0; i < PMeasureDataHolder.numRows() - 1; i++) {
			for(int j = 0; j <PMeasureDataHolder.numCols(); i++) {
				MeasureDataHolder.set(i + 1, j, PMeasureDataHolder.get(i, j));
			}
		}
		
		for(int i = 0; i < measurement.numRows(); i++) MeasureDataHolder.set(0, i, measurement.get(i));
		
		PMeasureDataHolder = MeasureDataHolder;
	}
	void predict(SimpleMatrix pState, SimpleMatrix pCovariance, SimpleMatrix stateChange, SimpleMatrix processNoiseCovariance, SimpleMatrix inputEffect, SimpleMatrix control) {
		//X, P, A, Q, B, U
		State = stateChange.mult(pState).plus(inputEffect.mult(control));
		Covariance = A.mult(pCovariance).mult(A.transpose()).plus(processNoiseCovariance);
	}
	void predict() {
		addStateData();
		Q = covariance(StateDataHolder, true);
		predict(State, Covariance, A, Q, B, Control);
	}
	void update(SimpleMatrix covariance, SimpleMatrix outputAdaptation, SimpleMatrix measurement, SimpleMatrix measureCovariance) {
		KalmanGain = covariance.mult(outputAdaptation.transpose()).mult((outputAdaptation.mult(covariance).mult(outputAdaptation.transpose()).plus(measureCovariance)).invert());
		State = State.plus(KalmanGain.mult(measurement.minus(outputAdaptation.mult(State))));
		Covariance = (SimpleMatrix.identity(KalmanGain.numRows()).minus(KalmanGain.mult(outputAdaptation))).mult(Covariance);
	}
	void update(SimpleMatrix measurement) {
		addMeasureData(measurement);
		R = covariance(MeasureDataHolder, true);
		update(Covariance, C, measurement, R);
	}
	SimpleMatrix covariance(SimpleMatrix Data, boolean related) {
		//Data comes in with each column as an attribute
		SimpleMatrix Covariances = new SimpleMatrix(Data.numCols(), Data.numCols());
		SimpleMatrix ConstMatrix = new SimpleMatrix(Data.numRows(), Data.numRows());
		SimpleMatrix Variance = new SimpleMatrix(Data.numRows(), Data.numCols());
		ConstMatrix.plus(1/((double)Data.numRows()));
		Variance = Data.minus(ConstMatrix.mult(Data));
		Covariances = Variance.transpose().mult(Variance);
		
		if(!related) {
			for(int i = 0; i < Covariances.numRows(); i ++) {
				for(int j = 0; j < Covariances.numCols(); j ++) {
					if(!(i==j)) Covariances.set(i, j, 0);
				}
			}
		}
		
		return Covariances;
	}
}
