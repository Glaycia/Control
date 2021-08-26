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
	
<<<<<<< HEAD
	LQRPicker(SimpleMatrix StateErrorTolerance, SimpleMatrix ControlErrorTolerance, double StateControlCostRatio){
		Q = new SimpleMatrix(StateErrorTolerance.numRows(), StateErrorTolerance.numRows());
		R = new SimpleMatrix(ControlErrorTolerance.numRows(), ControlErrorTolerance.numRows());
		
		Q.setTo(ColumnToDiagCost(StateErrorTolerance));
		R.setTo(ColumnToDiagCost(ControlErrorTolerance));
=======
	LQRPicker(SimpleMatrix StateErrorTolerance, SimpleMatrix ControlErrorTolerance, double StateControlCostRatio){	
		Q = (ColumnToDiagCost(StateErrorTolerance));
		R = (ColumnToDiagCost(ControlErrorTolerance));
>>>>>>> 47f464a36466a6f8e4d2fd3e171f1d29f3c1af29
		
		R.scale(StateControlCostRatio); //Bigger Cost Means Control Is Expensive, Smaller Cost Means Non Reference State is Expensive
	}
}
