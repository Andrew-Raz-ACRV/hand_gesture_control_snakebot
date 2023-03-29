#pragma once

//*******************************************//
//**************HAND FUNCTIONS***************//
//*******************************************//

bool isCorrectHandGesture(float index_z, float hand_radius) {
	if ((abs(index_z) <= 1) && (hand_radius <= 60) && (hand_radius != 0)) {
		return true; //Correct hand gesture
	}
	else { return false; } //Lost hand
}

bool isForcepsClosed(float Thumb_yaw, float Index_yaw, float Thumb_pitch, float Index_pitch, float pinchDistance, float hand_radius) {
	float difference_yaw = abs(Index_yaw - Thumb_yaw);
	float difference_pitch = abs(Index_pitch - Thumb_pitch);
	if ((pinchDistance <= 60) && (pinchDistance != 0)) {
		return true; //Closed
	}
	else {
		return false; //Open
	}
}

//******************************************************//
//************TRANSLATION AND ROTATION******************//
//******************************************************//
struct DesiredPosition {
	float X; float Y; float Z; float pitch; float roll;
};

DesiredPosition rotate2newOrigin(float XL, float YL, float ZL, float pitchL, float rollL, float X_start, float Y_start, float Z_start, float Xcr, float Ycr, float Zcr, float XL_pre, float YL_pre, float ZL_pre) {
	float X; float Y; float Z; float pitch; float roll;
	//Rotation Constants
	float S = sin(pitch_rotation); float C = cos(pitch_rotation);
	//X_relative = XLnew - Xstart
	float X_rel = XL - X_start;
	float Y_rel = YL - Y_start;
	float Z_rel = ZL - Z_start;

	//Deconstructed Translation and Rotation Matrix equations
	X = -X_rel;
	Y = C*Y_rel - S*Z_rel;
	Z = -S*Y_rel - C*Z_rel;
	pitch = -pitchL - pitch_rotation;
	roll = -rollL;

	//Scale down
	X = X / scale;
	Y = Y / scale;
	Z = Z / scale;

	//Add forward Kinematics at start:
	X += Xcr;
	Y += Ycr;
	Z += Zcr;

	//Output

	return{ X,Y,Z,pitch,roll };

	//X_rel=XL-XL_start; transform X_rel to intermedia frame; apply scaling; X_desired=X_scaled+x_c
}

//******************************************************//
//************CHECK TUBE CLIPPPING LIMIT FUNCTION*******//
//******************************************************//
bool isClipped(float tube1, float tube2, float tube3) {
	if ((tube1>(q1_max + 10)) || (tube1<-1) || (tube2>(q2_max + 10)) || (tube2<-1) || (tube3>(q3_max + 10)) || (tube3<-1)) {
		return true; //will clip
	}
	else {
		return false; //will not clip
	}
}

//******************************************************//
//***********SATURATE TUBE CLIP LIMIT FUNCTION**********//
//******************************************************//
struct SaturatedQ {
	float tube1; float tube2; float tube3;
};

SaturatedQ saturate(float tube1, float tube2, float tube3) {
	if (tube1 < 0) { tube1 = 0; }
	else if (tube1 > q1_max) { tube1 = q1_max; }
	else { tube1 = tube1; }

	if (tube2 < 0) { tube2 = 0; }
	else if (tube2 > q2_max) { tube2 = q2_max; }
	else { tube2 = tube2; }

	if (tube3 < 0) { tube3 = 0; }
	else if (tube3 > q3_max) { tube3 = q3_max; }
	else { tube3 = tube3; }

	return{ tube1,tube2,tube3 };
}

//*****************************************************//
//************CHECK WORKSPACE BOUNDARIES***************//
//*****************************************************//
bool isInWorkspace(float X, float Y, float Z) {

	float R = sqrt((X*X) + (Y*Y)); //Radial Distance
	float R_limit; //to define instantaneous radial limit

	if (Z < 0) { // Z inner boundary
		return false;
	}
	else if (Z <= Z1) { // Circular Part
		R_limit = r - sqrt(r*r - (Z*Z));
		if (R <= R_limit) { return true; }
		else { return false; }
	}
	else if (Z <= Z2) { // no q1 part
		R_limit = M*Z + c;
		if (R <= R_limit) { return true; }
		else { return false; }
	}
	else if (Z < Z3) { // to max all extrusions
		R_limit = R_limit2;
		if (R <= R_limit) { return true; }
		else { return false; }
	}
	else if (Z <= Z3) { // Z must be in the max area
		R_limit = M_m*Z + c_m;
		if (R >= R_limit2) { return false; }
		else if (R <= R_limit) { return false; }
		else { return true; }
	}
	else { //Z outer boundary
		return false;
	}
}

//*****************************************************//
//************PLACE IN WORKSPACE BOUNDARY***************//
//*****************************************************//
struct coordinates {
	float R; float Z;
};

coordinates put_in_workspace(float R, float Z) {
	float R_limit; //to define instantaneous radial limit

	if (Z < 0) { // Z inner boundary
		Z = 0;
	}
	else if (Z > Z3) { // Z outer boundary
		Z = Z3;
	}

	if (Z < Z1) { // Circular Part
		R_limit = r - sqrt(r*r - (Z*Z));
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z < Z2) { // no q1 part
		R_limit = M*Z + c;
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z < Z3) { // to max all extrusions
		R_limit = R_limit2;
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z <= Z3) { // Z must be in the max area
		R_limit = M_m*Z + c_m;
		if (R > R_limit2) {
			R = R_limit2;
		}
		else if (R < R_limit) {
			R = R_limit;
		}
	}
	return{ R,Z };
}

//**********************************************************//
//************XYZ PLACE IN WORKSPACE BOUNDARY***************//
//**********************************************************//
struct coordinatesXYZ {
	float X; float Y; float Z;
};

coordinatesXYZ XYZput_in_workspace(float X, float Y, float Z) {
	float R_limit; //to define instantaneous radial limit
	float R = sqrt((X*X) + (Y*Y)); //Radial Distance
	float theta = atan2(Y, X);

	if (Z < 0) { // Z inner boundary
		Z = 0;
	}
	else if (Z > Z3) { // Z outer boundary
		Z = Z3;
	}

	if (Z < Z1) { // Circular Part
		R_limit = r - sqrt(r*r - (Z*Z));
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z < Z2) { // no q1 part
		R_limit = M*Z + c;
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z < Z3) { // to max all extrusions
		R_limit = R_limit2;
		if (R > R_limit) {
			R = R_limit;
		}
	}
	else if (Z <= Z3) { // Z must be in the max area
		R_limit = M_m*Z + c_m;
		if (R > R_limit2) {
			R = R_limit2;
		}
		else if (R < R_limit) {
			R = R_limit;
		}
	}
	X = R*cos(theta);
	Y = R*sin(theta);
	return{ X, Y, Z };
}

//*****************************************************//
//************CHECK ACCURACY FUNCTION*******************//
//*****************************************************//
bool isnotaccurate(float R, float z, float q1, float q2, float q3) {
	//float R_actual = q3*sin(q2 / r) + r*(1 - cos(q2 / r)); yet to be implimented
	float z_actual = q1 + r*sin(q2 / r) + q3*cos(q2 / r);

	if (abs(z - z_actual)>1) {
		return true;
	}
	else {
		return false;
	}
}

//*****************************************************************//
//*******************KILL PROGRAM FUNCTION*************************//
//*****************************************************************//
bool operating(int life) {
	//PROGRAM Operates until it takes too long
	if (life >= 1000) {
		return false;
	}
	else {
		return true;
	}
}