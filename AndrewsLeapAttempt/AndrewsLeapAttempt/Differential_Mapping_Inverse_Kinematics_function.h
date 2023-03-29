#pragma once


//***************************************************************************//
//************DIFFERENTIAL MAPPING INVERSE KINEMATICS CODE*******************//
//***************************************************************************//
struct MotorQ {
	float q1; float q2; float q3; float q4; float q5; float q6; float q7; bool isvalid; int k;
};

MotorQ Differential_inverse_Kinematics_code(float dX, float dY, float dZ, float dP, float roll, float q1c, float q2c, float q3c, float q6c, int k, bool isopen) {
	float q1; float q2; float q3; float q4; float q5; float q6; float q7; bool isvalid = true;

	//Direct Motor Solutions:
	q7 = roll - q7_offset;
	q5 = 0;
	if (isopen == true) {
		q4 = -6000;
	}
	else { q4 = 0; }

	//Vector of CURRENT motor values:
	Vector4f Q;
	Q << q1c, q2c, q3c, q6c;
	//Vectors of the max and min limits:
	Vector4f Qmax;
	Qmax << q1_max, q2_max, q3_max, q6_max;
	Vector4f Qmin;
	Qmin << q1_min, q2_min, q3_min, q6_min;

	//*************ITERATIVE LOOP TO SOLVE ACCURATE Q by convergence**************//
	for (int i = 1; i <= 1; i++) { //Only 1 loop (real time) required

								   //Compute Current Jacobian:
		Matrix4f J;
		J(0, 0) = 0;   J(0, 1) = cos(Q(3, 0))*(sin(Q(1, 0) / r) + (Q(2, 0)*cos(Q(1, 0) / r)) / r);
		J(1, 0) = 0;   J(1, 1) = sin(Q(3, 0))*(sin(Q(1, 0) / r) + (Q(2, 0)*cos(Q(1, 0) / r)) / r);
		J(2, 0) = 1;   J(2, 1) = cos(Q(1, 0) / r) - (Q(1, 0)*sin(Q(1, 0) / r)) / r, cos(Q(1, 0) / r);
		J(3, 0) = 0;   J(3, 1) = (sin(Q(3, 0))*(tan((Q(1, 0) / r))*tan((Q(1, 0) / r)) + 1)) / (r * (tan((Q(1, 0) / r)) * tan((Q(1, 0) / r)) * sin(Q(3, 0)) * sin(Q(3, 0)) + 1));

		J(0, 2) = sin(Q(1, 0) / r)*cos(Q(3, 0));    J(0, 3) = sin(Q(3, 0))*(r*(cos(Q(1, 0) / r) - 1) - Q(2, 0)*sin(Q(1, 0) / r));
		J(1, 2) = sin(Q(1, 0) / r)*sin(Q(3, 0));    J(1, 3) = -cos(Q(3, 0))*(r*(cos(Q(1, 0) / r) - 1) - Q(2, 0)*sin(Q(1, 0) / r));
		J(2, 2) = cos(Q(1, 0) / r);					J(2, 3) = 0;
		J(3, 2) = 0;								J(3, 3) = 0;

		//Compute transpose
		Matrix4f Jt = J.transpose();

		//Compute deltaX
		Vector4f deltaX;
		deltaX << dX, dY, dZ, dP;

		//Clamp Magnitude to reduce oscillation error:
		float error_magnitude = deltaX.norm();
		//Error maximum 
		float Dmax = q1_max / 8; //originally 8
		//Saturate Magnitude of error mainly for q6 as it is sensitive to error
		if (error_magnitude > Dmax) {
			deltaX = Dmax * (deltaX / error_magnitude);
		}
		//else continue using deltaX

		//Compute the lambda factor
		Matrix4f Lambda = Matrix4f::Identity();
		Vector4f W;
		W << 1, 1, 1, 10;
		int p = 2; int c = 1; float num; float den; float L;

		for (int i = 0; i <= 3; i++) {
			num = 2 * Q(i, 0) - Qmax(i, 0) - Qmin(i, 0);
			den = Qmax(i, 0) - Qmin(i, 0);
			L = c*(num / den);
			Lambda(i, i) = L*L + W(i, 0);
		}

		//Damped equation: J_damped
		Matrix4f J_damped = (J*Jt + Lambda);

		//Solve the New motor change
		Vector4f dQ = Jt * J_damped.inverse() * deltaX;

		//Output new motor value:
		Q(0, 0) = Q(0, 0) + dQ(0, 0);
		Q(1, 0) = Q(1, 0) + dQ(1, 0);
		Q(2, 0) = Q(2, 0) + dQ(2, 0);
		Q(3, 0) = Q(3, 0) + dQ(3, 0);

		//Protect new motor values from joint limit damage:
		if (Q(0, 0) > q1_max) {
			Q(0, 0) = q1_max;
		}
		else if (Q(0, 0) < q1_min) {
			Q(0, 0) = q1_min;
		}

		if (Q(1, 0) > q2_max) {
			Q(1, 0) = q2_max;
		}
		else if (Q(1, 0) < q2_min) {
			Q(1, 0) = q2_min;
		}

		if (Q(2, 0) > q3_max) {
			Q(2, 0) = q3_max;
		}
		else if (Q(2, 0) < q3_min) {
			Q(2, 0) = q3_min;
		}
	}

	//*********OUTPUT FINAL*******//
	q1 = Q(0, 0)- q1_offset;
	q2 = Q(1, 0)- q2_offset;
	q3 = Q(2, 0)- q3_offset;
	q6 = Q(3, 0) - q6_offset;

	return{ q1*mm2counts,q2*mm2counts,q3*mm2counts,q4,q5*radians2counts,q6*radians2counts,q7*radians2counts,isvalid,k };
}