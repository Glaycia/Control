import LQR.DCVController;

public class GenerateDCMController {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		DCVController testMotor = new DCVController(10, 0.1, 0.01, 1, 0.5);
		
		testMotor.GainSetter(1, 0.1, 5, 3);
		
		System.out.println("CONTROLLER: ");
		testMotor.DCSSC.K.print();
		System.out.println("STATE MAT: ");
		testMotor.DCSSC.A.print();
		System.out.println("CONTROL MAT: ");
		testMotor.DCSSC.B.print();
	}

}
