import org.ejml.simple.SimpleMatrix;

import LQR.DCVController;

public class GenerateDCMController {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		DCVController testMotor = new DCVController(1, 0.1, 0.01, 1, 0.5);
		
		testMotor.GainSetter(1, 100, 12, 2);
		
		System.out.println("CONTROLLER: ");
		testMotor.DCSSC.K.print();
		System.out.println("STATE MAT: ");
		testMotor.DCSSC.A.print();
		System.out.println("CONTROL MAT: ");
		testMotor.DCSSC.B.print();
		
//		System.out.println(testMotor.returnVoltage(0, 0));
//		System.out.println(testMotor.returnVoltage(0, -5));
//		System.out.println(testMotor.returnVoltage(0, -10));
//		System.out.println(testMotor.returnVoltage(0, -15));
//		System.out.println(testMotor.returnVoltage(0, -20));
//		
//		System.out.println(testMotor.returnVoltage(-5, 0));
//		System.out.println(testMotor.returnVoltage(-5, -5));
//		System.out.println(testMotor.returnVoltage(-5, -10));
//		System.out.println(testMotor.returnVoltage(-5, -15));
//		System.out.println(testMotor.returnVoltage(-5, -20));
		System.out.println(testMotor.returnVoltage(0, 90));
	}

}
