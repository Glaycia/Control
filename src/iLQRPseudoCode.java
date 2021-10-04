
public class iLQRPseudoCode {
	/*
	//ArrayList StateTraj
	//ArrayList ControlTraj
	void backwardPass() {
		// Calculate first order cost to go (38)
		// Calculate second order cost to go (39)
		
		for(int i = horizon - 1; i > 0; i --) {
			
			//Compute to go hessians and gradients
			
			if(hessian of control matrix is positive semidefinite) {
				//K, d, cV
				decrease regularization term
			}else {
				increase regularization term, backwardpass();
			}
		}
		
	}
	void fwdPass() {
		init first state, alpha = 1,  update cost
		for(int i = 0; i < horizon - 1; i ++) {
			newnominalcontrol = old + k(optim state - current state) + alpha * d;
			stepstate = a * currentstate + b * newnominalcontrol;
			
		}
		
		update cost
		
		if(j statisfies line search conditions) {
			update x and u
		}else {
			decrease alpha;
		}
	}
	
	void ilqr() {
		
		x0, tolerance
		init X from U;
		
		do {
			find J
			fwd pass
			backwards pass
		}while(J - oldJ  > tolerance)
		}
	
	return stuff;
	}
*/
}
