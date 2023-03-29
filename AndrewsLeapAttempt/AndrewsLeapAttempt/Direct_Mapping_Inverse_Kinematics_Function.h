#pragma once
//****************STRUCTURE OF SOLUTION CHANGE*****************//
struct solution {
	float dq1; float dq2; float dq3;
};


//*****************************************************//
//************DIRECT MAPPING INVERSE KINEMATICS CODE*******************//
//*****************************************************//
struct MotorQ {
	float q1; float q2; float q3; float q4; float q5; float q6; float q7; bool isvalid; int k;
};

MotorQ inverse_Kinematics_code(float X, float Y, float Z, float pitch, float yaw, float roll, float q6_previous, float q2_c, float q3_c, float q1_c, int k, bool isopen) {
	float q1; float q2; float q3; float q4; float q5; float q6; float q7; bool isvalid;

	int Solution_type = 0;
	// Structure of solutions
	//solution G; solution m1; solution m2; solution M1; solution M2;
	solution m; solution Mx;

	float R = sqrt((X*X) + (Y*Y)); //Radial Distance
								   //Saturate R and Z:
	coordinates Coordinates = put_in_workspace(R, Z);
	R = Coordinates.R;
	Z = Coordinates.Z;

	//Direct Motor Solutions:
	q7 = roll - q7_offset;

	q6 = (atan2(Y, X) - q6_offset) + (k * 2 * PI); //initial q6 computation (k = 0 at start)
												   // Logic with k: wind up/down k (defines periods in the rotation) to eliminate +180/-180 singularity - threshold diff of 2 radians
	if ((q6 - (q6_previous / radians2counts)) >= 2) {
		k--; //wind up
	}
	else if ((q6 - (q6_previous / radians2counts)) <= -2) {
		k++; //wind down
	}
	q6 = (atan2(Y, X) - q6_offset) + (k * 2 * PI); //Adjust q6 with the new wind up

	q5 = 0;
	if (isopen == true) {
		q4 = -6000;
	}
	else { q4 = 0; }

	//SOLVING THE OTHER 3 TUBES

	//1 Tube SOLUTION - SINGULAR ******************//
	if (R == 0) {
		q2 = 0;
		q3 = 0.5*Z;
		q1 = 0.5*Z;
		if (q1 > q1_max) {
			q1 = q1_max;
			q3 = Z - q1_max;
		}
		isvalid = true;
		Solution_type = 1;
		//std::cout << "  TUBE 1 SOLUTION USED ";
	}
	else {
		//GENERAL 3 TUBE SOLUTION**********************************************************//
		//q2 = r*atan2(R, (Y / tan( pitch ))); //original solution
		q2 = r*atan2((sqrt(tan(pitch)*tan(pitch) + tan(yaw)*tan(yaw))), 1);
		float C = cos(q2 / r);
		float S = sin(q2 / r);
		q3 = (R - (r*(1 - C))) / S;
		q1 = Z - q3*C - r*S;
		isvalid = true;
		Solution_type = 2;
		//G.dq1 = q1-q1_c; G.dq2 = q2-q2_c; G.dq3 = q3-q3_c;
		//std::cout << "  GENERAL SOLUTION USED ";

		//CHECK GENERAL SOLUTION
		if (isClipped(q1, q2, q3)) {
			//Must use other method

			//MAXIMUM CURVATURE SOLUTIONS*********************************************************//
			if (true || (abs(pitch) > (q2_max / r)) || (abs(yaw) > (q2_max / r))) {

				if (R >= (r*(1 - cos(q2_max / r)))) {
					//MAXIMUM CURVATURE 2 SOLUTION
					q2 = q2_max;
					q3 = (R - r*(1 - cos(q2_max / r)) / (sin(q2_max / r)));
					q1 = Z - q3*cos(q2_max / r) - r*sin(q2_max / r);
					Solution_type = 6;
					isvalid = true;
					//M2.dq1 = q1 - q1_c; M2.dq2 = q2 - q2_c; M2.dq3 = q3 - q3_c;
				}
				else {
					//MAXIMUM CURVATURE 1 SOLUTION
					q3 = 0;
					q2 = r* acos(1 - R / r);
					q1 = Z - r*sin(q2 / r);
					Solution_type = 3;
					isvalid = true;
					//M1.dq1 = q1 - q1_c; M1.dq2 = q2 - q2_c; M1.dq3 = q3 - q3_c;
				}
				isvalid = true;
				Mx.dq1 = q1 - q1_c; Mx.dq2 = q2 - q2_c; Mx.dq3 = q3 - q3_c;
			}

			//else default to min solution
			//Check Max solution
			if (isClipped(q1, q2, q3)) {
				//MINIMUM CURVATURE SOLUTION 1  *************************************************//
				//Solve the quadratic
				float a = ((R - r)*(R - r) / (Z*Z)) + 1;
				float b = 2 * (R - r)*r / (Z*Z);
				float c = r*r / (Z*Z) - 1;
				float cosine_solution1 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
				float cosine_solution2 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
				//Pick solution
				if (cosine_solution1<1 || cosine_solution1>0) {
					//Use cosine_solution1
					q2 = r*acos(cosine_solution1);
					q3 = (R - r*(1 - cosine_solution1)) / sqrt(1 - cosine_solution1*cosine_solution1);
					q1 = 0;
					//m1.dq1 = q1 - q1_c; m1.dq2 = q2 - q2_c; m1.dq3 = q3 - q3_c;
					if (isClipped(q1, q2, q3)) {
						isvalid = true;
						SaturatedQ ClippedSolution = saturate(q1, q2, q3);
						q1 = ClippedSolution.tube1; q2 = ClippedSolution.tube2; q3 = ClippedSolution.tube3;
						Solution_type = 0;
					}
					else {
						isvalid = true;
						Solution_type = 4;
					}
				}
				else if (cosine_solution2<1 || cosine_solution2>0) {
					//Use cosine_solution2 NOTE: This solution 1 is always correct in practise compared to this one!
					q2 = r*acos(cosine_solution2);
					q3 = (R - r*(1 - cosine_solution2)) / sqrt(1 - cosine_solution2*cosine_solution2);
					q1 = 0;
					//m1.dq1 = q1 - q1_c; m1.dq2 = q2 - q2_c; m1.dq3 = q3 - q3_c;
					if (isClipped(q1, q2, q3)) {
						isvalid = true;
						SaturatedQ ClippedSolution = saturate(q1, q2, q3);
						q1 = ClippedSolution.tube1; q2 = ClippedSolution.tube2; q3 = ClippedSolution.tube3;
						Solution_type = 0;
					}
					else {
						isvalid = true;
						Solution_type = 4;
					}
				}
				else {
					//failure
					isvalid = false;
					Solution_type = -1;
					std::cout << "Computational FAILURE NO SOLUTION "; //Note: in practise this error never occurs
				}

				if (q3 > q3_max) {
					//MINIMUM CURVATURE SOLUTION 2
					//Solve the quadratic
					float a = ((r*r) / (q3_max*q3_max)) + 1;
					float b = 2 * (R - r)*r / (q3_max*q3_max);
					float c = (((R - r)*(R - r)) / (q3_max*q3_max)) - 1;
					float cosine_solution1 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
					float cosine_solution2 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
					//Pick solution
					if (cosine_solution1<1 || cosine_solution1>0) {
						//Use cosine_solution1
						q2 = r*acos(cosine_solution1);
						q3 = q3_max;
						q1 = Z - q3_max*cosine_solution1 - r*sqrt(1 - (cosine_solution1*cosine_solution1));
						//m2.dq1 = q1 - q1_c; m2.dq2 = q2 - q2_c; m2.dq3 = q3 - q3_c;
						if (isClipped(q1, q2, q3)) {
							isvalid = true;
							SaturatedQ ClippedSolution = saturate(q1, q2, q3);
							q1 = ClippedSolution.tube1; q2 = ClippedSolution.tube2; q3 = ClippedSolution.tube3;
							Solution_type = 0;
						}
						else {
							isvalid = true;
							Solution_type = 5;
						}
					}
					else if (cosine_solution2<1 || cosine_solution2>0) {
						//Use cosine_solution2 NOTE: This solution 1 is always correct in practise compared to this one!
						q2 = r*acos(cosine_solution2);
						q3 = q3_max;
						q1 = Z - q3_max*cosine_solution2 - r*sqrt(1 - (cosine_solution2*cosine_solution2));
						//m2.dq1 = q1 - q1_c; m2.dq2 = q1 - q1_c; m2.dq3 = q1 - q1_c;
						if (isClipped(q1, q2, q3)) {
							isvalid = true;
							SaturatedQ ClippedSolution = saturate(q1, q2, q3);
							q1 = ClippedSolution.tube1; q2 = ClippedSolution.tube2; q3 = ClippedSolution.tube3;
							Solution_type = 0;
						}
						else {
							isvalid = true;
							Solution_type = 5;
						}
					}
					else {
						//failure *************************************************************************************//
						isvalid = false;
						Solution_type = -1;
						std::cout << "Computational FAILURE NO SOLUTION "; //Note: in practise this error never occurs thus there is always a solution
					}
				}
				m.dq1 = q1 - q1_c; m.dq2 = q2 - q2_c; m.dq3 = q3 - q3_c;

			}

		}

	}
	/*
	//REDUCE JUMP FACTOR
	if ((Solution_type == 3)|| (Solution_type == 6) || (Solution_type == 4) || (Solution_type == 5) || (Solution_type == 0)) {
	if (abs(m.dq1) < abs(Mx.dq1)) {
	q1 = m.dq1 + q1_c; q2 = m.dq2 + q2_c; q3 = m.dq3 + q3_c;
	}
	else {
	q1 = Mx.dq1 + q1_c; q2 = Mx.dq2 + q2_c; q3 = Mx.dq3 + q3_c;
	}
	}
	*/



	//ENSURE Z-position is accurate:
	if (isnotaccurate(R, Z, q1, q2, q3)) {
		float z_actual = q1 + r*sin(q2 / r) + q3*cos(q2 / r);
		q1 = Z - z_actual;
	}

	switch (Solution_type) {
	case 1:
		std::cout << "  TUBE 1 SOLUTION USED ";
		isvalid = true;
		break;
	case 2:
		std::cout << "  GENERAL SOLUTION USED ";
		isvalid = true;
		break;
	case 3:
		std::cout << "  MAXIMUM1 SOLUTION USED ";
		isvalid = true;
		break;
	case 6:
		std::cout << "  MAXIMUM2 SOLUTION USED ";
		isvalid = true;
		break;
	case 4:
		std::cout << "  Minimum1 SOLUTION USED ";
		isvalid = true;
		break;
	case 5:
		std::cout << "  Minimum2 SOLUTION USED ";
		isvalid = true;
		break;
	case 0:
		std::cout << " Minimum Clipped SOLUTION ";
		isvalid = true;
		break;
	default: //type 0
		std::cout << " ERROR ERROR ERROR ";
		isvalid = false;
		break;
	}
	return{ (q1-q1_offset)*mm2counts,(q2 - q2_offset)*mm2counts,(q3 - q3_offset)*mm2counts,q4,q5*radians2counts,q6*radians2counts,q7*radians2counts,isvalid,k };
}
