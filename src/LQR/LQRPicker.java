package LQR;
import org.ejml.simple.SimpleMatrix;

public class LQRPicker {
	SimpleMatrix Q;
	SimpleMatrix R;
	
	SimpleMatrix ColumnToDiagCost(SimpleMatrix Column) {
		SimpleMatrix Square = new SimpleMatrix(Column.numRows(), Column.numRows());
		for(int i = 0; i < Column.numRows(); i++) {
			Square.set(i, i, 1/(Column.get(i, 0) * Column.get(i, 0)));
		}
		//Square.print();
		return Square;
	}
	
	LQRPicker(SimpleMatrix StateErrorTolerance, SimpleMatrix ControlErrorTolerance, double StateControlCostRatio){
		Q = new SimpleMatrix(StateErrorTolerance.numRows(), StateErrorTolerance.numRows());
		R = new SimpleMatrix(ControlErrorTolerance.numRows(), ControlErrorTolerance.numRows());
		
		Q.setTo(ColumnToDiagCost(StateErrorTolerance));
		R.setTo(ColumnToDiagCost(ControlErrorTolerance));
		
		R.scale(StateControlCostRatio); //Bigger Cost Means Control Is Expensive, Smaller Cost Means Non Reference State is Expensive
	}
}
