import org.ejml.simple.SimpleMatrix;

public class MatrixTest {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		SimpleMatrix A = new SimpleMatrix(1,1);
		SimpleMatrix B = new SimpleMatrix(1,1);
		SimpleMatrix C = A;//new SimpleMatrix(A);
		A.set(0, 1);
		C.set(0, 2);
		System.out.println(A == B);
		System.out.println(A.equals(B));
		System.out.println(A == C);
		A.print();
		System.out.println(A == C);
		C.print();
		
	}

}
