// AndrewsLeapAttempt.cpp : Defines the entry point for the console application.
// USER INFORMATION IF PROBLEMS OCCUR:
// Turn on PMAC before plugging the ethernet cable into the computer
// Remember to always run Visual studio under Administrator and run Pewin32Pro under Administrator. If PMAC is not connecting at this stage try restarting the computer.
// Plug in the Leap Motion Controller via USB. Check that is on by checking if the red LED lights are on inside it.
// You can check the Leap motion controller quality by running the Leap motion visualizer. If this says it is not connected you may need to reinstall the drivers.

//******************************DIRECT OR DIFFERENTIAL MAPPING CONTROL OF A SNAKE ROBOT PROJECT CODE***************//
// To change to differential, comment the direct mapping file and uncomment the differential mapping file.
// Then comment the lines saying direct mapping

//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS
//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS
//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS

//***************INCLUDE FILES***************//
#include "afx.h"
#include "MyPMAC.h"
#include "myRuntime.h"

#include <math.h>

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string> 
#include <string.h>
#include "Leap.h"
#include <windows.h>
#include <Eigen/Dense> 

#include <ctime>
#include <stdio.h>
//Having troubles connecting to Eigen after copying this project? Go to the Project Properties and update the additional directories


using namespace Leap;
using namespace Eigen; //::MatrixXd;

					   //****** RECORDING TO CSV *******//
bool recording = false;
bool forceps_activate = true; //Do not record with the NDI if the forceps are open
bool Direct_mapping = false; //Set this true for direct mapping also switch commenting the include file and find and switch the function below called inverse_Kinematics_code  

//************** PMAC CLASS **************//

class CMyPMAC
{
public:
	void PmacProcess(LPTSTR wParam, LPCTSTR lParam);
	void PmacInit();
	CMyPMAC();
	virtual ~CMyPMAC();

private:
	DWORD dwDevice;
};

CMyPMAC* m_MyPMAC = new CMyPMAC;


//********************************************************
//GLOBAL CONSTANTS REGARDING THE SNAKE ROBOT AND WORKSPACE
//********************************************************

//curvature of tube 2
const float r = 173.58;	//edited by LW 20170308-2:40pm

//conversion to cts
const float radians2counts = 3200 * 50 / 15 / (2 * PI);
const float mm2counts = 400 * 50 / 15;

//tube approximate maximum extrusion in mm (RELATIVE)
const float q1_max = 103000 / mm2counts; //77.25mm
const float q2_max = 110000 / mm2counts; //82.5mm
const float q3_max = 130000 / mm2counts; //97.5mm
const float q6_max = 4 * PI;
const float q6_min = -4 * PI;

//Rotation motor initial angle and distance offsets from origin:
const float q1_offset =  18; //7mm18
const float q2_offset =  20; //25mm20
const float q3_offset =  28; //30mm28
const float q6_offset = 90 * (PI / 180); //degrees to radians90 * (PI / 180)
const float q7_offset =  0; //degrees to radians0 * (PI / 180)
//Tube minimum
const float q1_min = 0; //q1_offset;
const float q2_min = 0; //q2_offset;
const float q3_min = 0; //q3_offset;


//Rotation and Translation constants
const float z_translation = -550;
const float y_translation = -280;
const float pitch_rotation = 13 * PI / 180; //rotation in degrees to radians //16 was good
const float scale = 2; //Scale	//edited LW 20170803-2:50pm

//WORKSPACE CONSTANTS
const float Z_max = q1_max + q3_max;
//Computing the check points:
const float Z1 = r*sin(q2_max / r); //Z Limits
const float Z2 = Z1 + q3_max * cos(q2_max / r);
const float Z3 = Z2 + q1_max;
const float R_limit1 = r*(1 - cos(q2_max / r)); //R Limits
const float R_limit2 = R_limit1 + q3_max*sin(q2_max / r);
const float M = (R_limit2 - R_limit1) / (Z2 - Z1); //Gradient constants
const float c = R_limit1 - M*Z1;
const float M_m = (R_limit2) / (Z3 - Z_max);
const float c_m = R_limit2 - M*Z3;

//********FILE OF FUNCTIONS REGARDING HAND GESTURE AND WORKSPACE*****//
#include "Hand_Gesture_Workspace_functions.h"

//********FILE OF DIRECT MAPPING INVERSE KINEMATICS*****//
//#include "Direct_Mapping_Inverse_Kinematics_function.h"

//********FILE OF DIFFERENTIAL MAPPING INVERSE KINEMATICS*****//
#include "Differential_Mapping_Inverse_Kinematics_function.h"



//*****************************************************//
//*******************MAIN CODE*************************//
//*****************************************************//
int main()
{
	//Define initial time step
	int time_K = 0;
	std::ofstream myfile_leap;
	std::ofstream myfile_PMAC;
	std::ofstream myfile_motors;

	if (recording) {
		//Begin recording data from the Leap:
		myfile_leap.open("LEAP_DATA.csv");
		myfile_leap << "Test Leap Motion Data\n";
		myfile_leap << "Time min, sec, ms,X,Y,Z,Roll,Pitch,Yaw,time from start\n";

		//Begin recording data from the Leap:
		myfile_PMAC.open("PMAC_DATA.csv");
		myfile_PMAC << "Test PMAC Forward Kinematics Motion Data\n";
		myfile_PMAC << "Time min, sec, ms,X,Y,Z,Roll,Pitch,Yaw,time from start\n";

		//Begin recording data from the NDI
		myfile_motors.open("motor_DATA.csv");
		myfile_motors << "Test motor Data\n";
		myfile_motors << "q1,q2,q3,q6,q7\n";
	}

	//Initialise NDI
	float X_mdi = 0; float Y_mdi = 0; float Z_mdi = 0; float Roll_mdi = 0; float Pitch_mdi = 0; float Yaw_mdi = 0;

	//Obtain the leap motion controller object
	Controller controller;

	//INITIALISE MAIN VARIABLES
	float X = 0; float Y = 0; float Z = 0;
	float pitch = 0; float yaw = 0; float roll = 0;
	//Leap
	float XL = 0; float YL = 0; float ZL = 0;

	float dX; float dY; float dZ; float dP;
	//ORIGIN RELATIVE CHANGE
	float Xr = 0; float Yr = 0; float Zr = 0;

	//Starting point
	float XL_start = 0; float YL_start = 0; float ZL_start = 0;
	//Previous point 
	float XL_pre = 0; float YL_pre = 0; float ZL_pre = 0;

	//other initialisation
	int k_wind_up = 0; int life = 0;
	bool isopen = false;
	
	bool STOP = false;
	bool Tracking = true; bool Tracking_pre = false;

	//Initialise Current MOTOR VALUES
	float q1c = 0; float q2c = 0; float q3c = 0; float q6c = 0; float q7c = 0;  float q6_pre = 0;
	float Xc = 0; float Yc = 0; float Zc = 0; float Pc = 0; float Rollc = 0; float YAWc = 0;
	float Xcr = 0; float Ycr = 0; float Zcr = 0;
	//******************************************************//
	//----------------initialise PMAC-----------------------//
	//******************************************************//

	TCHAR buf[256];
	CString str, strtemp;
	m_MyPMAC->PmacInit();
	m_MyPMAC->PmacProcess(buf, "i122=64 i222=64 i322=64 i422=64 i522=8 i622=8 i722=8"); //jog speed of axis 1 to axis 6
	m_MyPMAC->PmacProcess(buf, "#1j=0 #2j=0 #3j=0 #4j=10 #5j=0 #6j=0 #7j=0");
	m_MyPMAC->PmacProcess(buf, "#1j=50000 #2j=100000 #3j=150000 #4j=10 #5j=0 #6j=0 #7j=0");

	//******************************************************//
	// --------------Main Loop of Operation-----------------//
	//******************************************************//
	//Initialise time of loop
	double elapsed_secs = 0;
	
	SYSTEMTIME st;
	GetSystemTime(&st);
	std::cout << "Start time is: " << st.wMinute << " minutes " << st.wSecond << " seconds " << st.wMilliseconds << " miiliseconds\n";
	double time = 0;
	while (operating(life) == true) {
		clock_t begin = clock();

		if (recording) {
			//std::cout << "Time: " << time_K << " ";
			time_K++;
			//std::cout << "Loop duration: " << elapsed_secs << "  ";
			//SYSTEMTIME st;
			//GetSystemTime(&st);
			time = time + elapsed_secs;
			std::cout << "Time from start: " << time << "  ";

			//std::cout << "time is: " << st.wMinute << " minutes " << st.wSecond << " seconds " << st.wMilliseconds << " miiliseconds  ";
		}

		//if the sensor is connected
		if (controller.isConnected())
		{
			//******************************************************//
			//******************READING HAND DATA*******************//
			//******************************************************//

			//COLLECT FRAME of hand and finger data
			Frame frame = controller.frame(); //The latest frame
											  //Frame previous = controller.frame(1); //The previous frame maybe useful for differential mapping

											  //Initialise Vectors
			Vector average_index = Vector(); Vector average_thumb = Vector(); Vector average_palm_position = Vector();
			//Directions
			Vector ave_dir_thumb = Vector(); Vector ave_dir_index = Vector();

			//Initialise Hand reading
			HandList hands = frame.hands();
			Hand firstHand = hands[0]; // reads data only from the first hand that it sees
			Vector Palm_position = firstHand.palmPosition();

			//THUMB FINGER: Average the finger tip position 
			Finger fingerToAverage = frame.fingers()[0]; //thumb
			int count = 0;
			for (int i = 0; i < 30; i++)
			{
				Finger fingersFromFrame = controller.frame(i).finger(fingerToAverage.id());
				if (fingersFromFrame.isValid()) {
					average_thumb += fingersFromFrame.tipPosition();
					ave_dir_thumb += fingersFromFrame.direction();
					count++;
				}
			}
			average_thumb /= count;
			ave_dir_thumb /= count;

			//INDEX FINGER: Average the finger tip position 
			fingerToAverage = frame.fingers()[1]; //index
			count = 0;
			for (int i = 0; i < 30; i++)
			{
				Finger fingersFromFrame = controller.frame(i).finger(fingerToAverage.id());
				if (fingersFromFrame.isValid()) {
					average_index += fingersFromFrame.tipPosition();
					ave_dir_index += fingersFromFrame.direction();
					count++;
				}
			}
			average_index /= count;
			ave_dir_index /= count;

			//*************OUTPUT OF THE READINGS**************//

			//Index directions:
			// 0 is yaw; 1 is pitch; 2 is roll;
			float index_z = ave_dir_index[1];
			float hand_radius = firstHand.sphereRadius();

			//CHECK HAND GESTURE
			if (isCorrectHandGesture(index_z, hand_radius) == true) {
				STOP = false;
				if (Tracking_pre == false) {
					Tracking = true;
				}

				std::cout << "Correct Output: ";
				//UPDATE READINGS
				//Hand angles in Radians
				pitch = firstHand.direction().pitch();
				yaw = firstHand.direction().yaw();
				roll = firstHand.palmNormal().roll();

				/* INDEX finger position in mm is a bit unstable for use
				X = average_index[0]; Y = average_index[1];Z = average_index[2];*/

				//average Palm Position in mm is a bit more stable
				Vector average_palm_position = Palm_position;
				X = average_palm_position[0];
				Y = average_palm_position[1];
				Z = average_palm_position[2];

				//Round the position to the nearest micro metre: (to reduce noise)
				X = (round(X * 1000)) / 1000;
				Y = (round(Y * 1000)) / 1000;
				Z = (round(Z * 1000)) / 1000;

				//Record position relative to leap:
				XL = X; YL = Y; ZL = Z;

				//******DEFINE RELATIVE POSITION*****// 
				if ((Tracking_pre == false) && (Tracking == true)) {
					//******************READ CURRENT MOTOR VALUES*************//
					m_MyPMAC->PmacProcess(buf, "#1p");
					q1c = round(atof(buf) / mm2counts);

					m_MyPMAC->PmacProcess(buf, "#2p");
					q2c = round((atof(buf) / mm2counts) - q1c);

					m_MyPMAC->PmacProcess(buf, "#3p");
					q3c = round((atof(buf) / mm2counts) - q1c - q2c);

					m_MyPMAC->PmacProcess(buf, "#6p");
					q6c = atof(buf) / radians2counts;

					//Adjust by offset
					q1c = q1c + q1_offset;
					q2c = q2c + q2_offset;
					q3c = q3c + q3_offset;
					q6c = q6c + q6_offset;

					//*******************FORWARD KINEMATICS at start of tracking********************//
					Xcr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*cos(q6c);
					Ycr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*sin(q6c);
					Zcr = q3c*cos(q2c / r) + r*sin(q2c / r) + q1c;

					//Record Start 
					XL_start = XL; YL_start = YL; ZL_start = ZL;

					//Define new origin change:
					//Xr = (X - Xc); Yr = (Y - Yc); Zr = (Z - Zc);
					//record XL, YL, ZL XL_start...


					Tracking_pre = true;
				}
				


				//Rotate and Translate Readings
				//DesiredPosition Desired = rotate2newOrigin(X, Y, Z, pitch, roll, Xr, Yr, Zr);
				DesiredPosition Desired = rotate2newOrigin(XL, YL, ZL, pitch, roll, XL_start, YL_start, ZL_start, Xcr, Ycr, Zcr, XL_pre,YL_pre,ZL_pre);

				//Update X,Y,Z based on the desired position
				X = Desired.X;
				Y = Desired.Y;
				Z = Desired.Z;
				pitch = Desired.pitch;
				roll = Desired.roll;


				//user interaction reset life counting
				life = 0;

				//*****************Variables for the opening of the forceps*************//
				float Thumb_yaw = ave_dir_thumb[0]; float Index_yaw = ave_dir_index[0];
				float Thumb_pitch = ave_dir_thumb[2]; float index_pitch = ave_dir_index[2];
				float pinchDistance = firstHand.pinchDistance();
				bool Current_state = isForcepsClosed(Thumb_yaw, Index_yaw, Thumb_pitch, index_pitch, pinchDistance, hand_radius);

				//CHECK IF FORCEPS ARE CLOSED OR NOT in the correct hand gesture
				if (pinchDistance != 0) {

					if ((Current_state == true)) {
						std::cout << " Forceps are closed ";
						isopen = false;
						m_MyPMAC->PmacProcess(buf, "#4j=10");
						
						
					}
					else {
						std::cout << " Forceps are open ";
						isopen = true;
						if (forceps_activate == true) {
							m_MyPMAC->PmacProcess(buf, "#4j=-3000");
						}
						
					}

				}

				//********WHEN THE INCORRECT HAND GESTURE OCCURS******//
			}
			else {
				std::cout << "Lost freeze motion: ";
				STOP = true;
				if (Tracking_pre == true) {
					Tracking_pre = false;
					//Record previous Leap data before losing track
					XL_pre = XL; YL_pre = YL; ZL_pre = ZL;
				}
				m_MyPMAC->PmacProcess(buf, "#1j/");
				m_MyPMAC->PmacProcess(buf, "#2j/");
				m_MyPMAC->PmacProcess(buf, "#3j/");
				m_MyPMAC->PmacProcess(buf, "#4j/");
				m_MyPMAC->PmacProcess(buf, "#6j/");
				m_MyPMAC->PmacProcess(buf, "#7j/");
				//No user interaction start counting the life period to resetting
				life++;

			}
			//******************READ CURRENT MOTOR VALUES*************//
			m_MyPMAC->PmacProcess(buf, "#1p");
			q1c = (atof(buf) / mm2counts);
			
			m_MyPMAC->PmacProcess(buf, "#2p");
			q2c = ((atof(buf) / mm2counts) - q1c);
			
			m_MyPMAC->PmacProcess(buf, "#3p");
			q3c = ((atof(buf) / mm2counts) - q1c - q2c);
			
			m_MyPMAC->PmacProcess(buf, "#6p");
			q6c = atof(buf) / radians2counts;

			//Adjust by offset
			q1c = q1c + q1_offset;
			q2c = q2c + q2_offset;
			q3c = q3c + q3_offset;
			q6c = q6c + q6_offset;

			m_MyPMAC->PmacProcess(buf, "#7p");
			q7c = atof(buf) / radians2counts;
			//Adjust q6 by offset
			q7c = q7c + q7_offset;
			
			
			//std::cout << " Current motors: " << " q1 " << q1c << " q2 " << q2c << " q3 " << q3c << " q6 " << q6c << "\n";

			//*******************FORWARD KINEMATICS********************//
			Xc = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*cos(q6c);
			Yc = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*sin(q6c);
			Zc = q3c*cos(q2c / r) + r*sin(q2c / r) + q1c;
			//Orientation
			Pc = atan(sin(q6c)*tan(q2c / r));
			YAWc = atan(cos(q6c)*tan(q2c / r));
			Rollc = q7c;

			if (Direct_mapping == false) {
				//DIFFERENTIAL MAPPING tailoring the differential values
				coordinatesXYZ Coordinates = XYZput_in_workspace(X, Y, Z);
				X = Coordinates.X; Y = Coordinates.Y; Z = Coordinates.Z;
				dX = X - Xc; dY = Y - Yc; dZ = Z - Zc; dP = pitch - Pc;
			}
			//*// Note Direct Mapping has put in workspace embedded in its algorithm

			//Record the Data
			if (recording) {
				//Record time and measurement
				myfile_leap << st.wMinute << "," << st.wSecond << "," << st.wMilliseconds << "," << X << "," << Y << "," << Z << "," << roll << "," << pitch << "," << yaw << "," << time << ",\n";
				myfile_PMAC << st.wMinute << "," << st.wSecond << "," << st.wMilliseconds << "," << Xc << "," << Yc << "," << Zc << "," << Rollc << "," << Pc << "," << YAWc << "," << time << ",\n";
				myfile_motors << q1c << "," << q2c << "," << q3c << "," << q6c << "," << q7c << ",\n";

			}
			//std::cout << " X= " << X << " Y= " << Y << " Z= " << Z << " P= " << pitch << "\n";
			//std::cout << " Xc= " << Xc << " Yc= " << Yc << " Zc= " << Zc << " Pc= " << Pc << "\n";

			//*******************SOLVE INVERSE KINEMATICS**************//

			//Check if the point is in the workspace or if the hand stop moving
			//if ((isInWorkspace(X, Y, Z) == true) && (STOP == false)) { //edited by LW 20170308-2:41pm
			if (STOP == false) {

				//DIFFERENTIAL MAPPING Compute motor values
				MotorQ Motor_input = Differential_inverse_Kinematics_code(dX, dY, dZ, dP, roll, q1c, q2c, q3c, q6c, k_wind_up, isopen);

				//DIRECT MAPPING Compute motor values
				//MotorQ Motor_input = inverse_Kinematics_code(X, Y, Z, pitch, yaw, roll, q6_pre, q2c, q3c, q1c, k_wind_up, isopen); //consider rounding or not rounding x y z prescision vs noise counts
				
				
				//q6 wind up:
				k_wind_up = Motor_input.k;

				//Allocate Previous Values for next loop
				q6_pre = Motor_input.q6;

				if (Motor_input.isvalid == true) {

					//Motor saturation filter: protects tubes by ensuring that tube 1, 2, 3 never goes to negative!
					if (Motor_input.q1 < q1_offset) {
						Motor_input.q1 = q1_offset;
					}
					if (Motor_input.q2 < q2_offset) {
						Motor_input.q2 = q2_offset;
					}
					if (Motor_input.q3 < q3_offset) {
						Motor_input.q3 = q3_offset;
					}

					//*******************SEND THE MOTOR COMMANDS*************//
					/*
					std::cout << " Give motors: "
					<< " q1 " << Motor_input.q1
					<< " q2 " << (Motor_input.q2 + Motor_input.q1)
					<< " q3 " << (Motor_input.q2 + Motor_input.q1 + Motor_input.q3)
					<< " q4 " << Motor_input.q4
					<< " q5 " << Motor_input.q5
					<< " q6 " << Motor_input.q6
					<< " q7 " << Motor_input.q7
					<< "\n";
					*/

					//std::cout << " Pitch " << (pitch * 180 / PI) << " X: " << X << " Y: " << Y << " Z: " << Z;
					//std::cout << " Yaw " << (yaw * 180 / PI);
					std::cout << "\n";

					strtemp.Format("%f", Motor_input.q1);
					str = _T("#1j=") + strtemp;
					m_MyPMAC->PmacProcess(buf, str);

					strtemp.Format("%f", (Motor_input.q2 + Motor_input.q1));
					str = _T("#2j=") + strtemp;
					m_MyPMAC->PmacProcess(buf, str);

					strtemp.Format("%f", (Motor_input.q2 + Motor_input.q1 + Motor_input.q3));
					str = _T("#3j=") + strtemp;
					m_MyPMAC->PmacProcess(buf, str);

					strtemp.Format("%f", Motor_input.q6);
					str = _T("#6j=") + strtemp;
					m_MyPMAC->PmacProcess(buf, str);

					strtemp.Format("%f", Motor_input.q7); //increase sensitivity of rotation
					str = _T("#7j=") + strtemp;
					m_MyPMAC->PmacProcess(buf, str);

				}
				else { //if inverse kinematic solution failed
					std::cout << "CANCELLED \n";
				}
			}
			else { // if outside workspace
				std::cout << "OUTSIDE WORKSPACE \n";
			}
		}
		else //if not connected
		{
			std::cout << "The Leap Motion Sensor is Not Connected at this current instance May reconnect soon...\n";
			//Leap motion not working start counting the life period to resetting
			life++; Sleep(10);
		}
		//Record completion time:
		clock_t end = clock();
		elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	}
	//Quit recording Leap motion data
	if (recording) {
		myfile_leap.close(); myfile_PMAC.close(); //myfile_NDI.close();
	};

	//RESET ROBOT
	std::cout << "OPERATION EXPIRED: RESETTING THE SNAKE ROBOT:"
		<< " q1 " << 0
		<< " q2 " << 0
		<< " q3 " << 0
		<< " q4 " << 0
		<< " q5 " << 0
		<< " q6 " << 0
		<< " q7 " << 0
		<< "\n";

	//*******************SEND THE RESET MOTOR COMMANDS*************//
	std::cout << "Program is Terminating now";
	m_MyPMAC->PmacProcess(buf, "#4j=10");
	Sleep(2000); //ensure forceps are closed before going into a tube
				 //Do rotation first:
	m_MyPMAC->PmacProcess(buf, "#6j=0");
	m_MyPMAC->PmacProcess(buf, "#7j=0");
	m_MyPMAC->PmacProcess(buf, "#5j=0");

	bool waiting_for_q6 = true;

	while (waiting_for_q6 == true) {
		m_MyPMAC->PmacProcess(buf, "#6p");
		q6c = atof(buf);
		if (abs(q6c) < 5) {
			//i.e. reading about 0cts in a +-5cts range
			waiting_for_q6 = false;
		}
	}
	//Now the tubes
	m_MyPMAC->PmacProcess(buf, "#3j=0");
	m_MyPMAC->PmacProcess(buf, "#2j=0");
	m_MyPMAC->PmacProcess(buf, "#1j=0");
	Sleep(2000); //wait a bit for this to be completed
				 //Redo if hits limit switch
	m_MyPMAC->PmacProcess(buf, "#3j=0");
	m_MyPMAC->PmacProcess(buf, "#2j=0");
	m_MyPMAC->PmacProcess(buf, "#1j=0");
	Sleep(2000);
	m_MyPMAC->PmacProcess(buf, "#3j=0");
	m_MyPMAC->PmacProcess(buf, "#2j=0");
	m_MyPMAC->PmacProcess(buf, "#1j=0");
	Sleep(1000);

	return 0;
}



