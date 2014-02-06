/***********************************************************************

    OSCeleton - OSC proxy for kinect skeleton data.
    Copyright (C) <2010>  <Sensebloom lda.>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

***********************************************************************/

#include <cstdio>
#include <csignal>

#include <XnCppWrapper.h>

#include <lo/lo.h>

#include "common.h"
#include <stdio.h>
#include <math.h>
#include <iostream>



char *ADDRESS = "127.0.0.1";
char *PORT = "7110";

#define OUTPUT_BUFFER_SIZE 1024*16
char osc_buffer[OUTPUT_BUFFER_SIZE];

char tmp[50]; //Temp buffer for OSC address pattern
int userID;
float jointCoords[3];
float jointOrients[9];
float eulerAngles[3];


float posConfidence;
float orientConfidence;

bool record = false;
bool mirrorMode = true;
bool sendRot = false;
bool filter = false;
bool preview = false;
bool raw = false;
bool sendOrient = false;
int nDimensions = 3;

void (*oscFunc)(lo_bundle*, char*) = NULL;

xn::Context context;
xn::DepthGenerator depth;
xn::DepthMetaData depthMD;
xn::UserGenerator userGenerator;
lo_address addr;


// Callback: New user was detected
void XN_CALLBACK_TYPE new_user(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("New User %d\n", nId);
	userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);

	lo_send(addr, "/new_user","i",(int)nId);
}



// Callback: An existing user was lost
void XN_CALLBACK_TYPE lost_user(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("Lost user %d\n", nId);

	lo_send(addr, "/lost_user","i",(int)nId);
}


// Callback: Started calibration
void XN_CALLBACK_TYPE calibration_started(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	printf("Calibration started for user %d\n", nId);
}



// Callback: Finished calibration
void XN_CALLBACK_TYPE calibration_ended(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		printf("Calibration complete, start tracking user %d\n", nId);
		userGenerator.GetSkeletonCap().StartTracking(nId);

		lo_send(addr, "/new_skel","i",(int)nId);
	}
	else {
		printf("Calibration failed for user %d\n", nId);
		userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

int jointPos(XnUserID player, XnSkeletonJoint eJoint) {

	XnSkeletonJointTransformation jointTrans;

	userGenerator.GetSkeletonCap().GetSkeletonJoint(player, eJoint, jointTrans);

	posConfidence = jointTrans.position.fConfidence;

	userID = -1;

	if (!raw)
	{
		jointCoords[0] = ((1280 - jointTrans.position.position.X) / 2560); //Normalize coords to 0..1 interval
		jointCoords[1] = ((960 - jointTrans.position.position.Y) / 1920); //Normalize coords to 0..1 interval
		jointCoords[2] = (jointTrans.position.position.Z * 7.8125 / 10000); //Normalize coords to 0..7.8125 interval
	}
	else
	{
		jointCoords[0] = jointTrans.position.position.X;
		jointCoords[1] = jointTrans.position.position.Y;
		jointCoords[2] = jointTrans.position.position.Z;
	}



	if (sendOrient)
	{
		orientConfidence = jointTrans.orientation.fConfidence;

		for (int i=0; i<9; i++)
		{
			jointOrients[i] = jointTrans.orientation.orientation.elements[i];
		}

		//jointsOrient =  (0:X1, 1:Y1, 2:Z1, 3:X2, 4:Y2, 5:Z2, 6:X3, 7:Y3, 8:Z3)
		if ( jointOrients[2] < +1)
		{
			if (jointOrients[2] > -1)
			{
				eulerAngles[1] = (asin(jointOrients[2]))*180/M_PI;
				eulerAngles[0] = (atan2(-jointOrients[5],jointOrients[8]))*180/M_PI;
				eulerAngles[2] = (atan2(-jointOrients[1],jointOrients[0]))*180/M_PI;
			}
        		else
       			{
				eulerAngles[1] = -90.0;//-M_PI/2;
				eulerAngles[0] = (-atan2(jointOrients[3],jointOrients[4]))*180/M_PI;
				eulerAngles[2] = 0;
			}
		}
		else
		{
			eulerAngles[1] = 90.0;//+M_PI/2;
			eulerAngles[0] = (atan2(jointOrients[3],jointOrients[4]))*180/M_PI;
			eulerAngles[2] = 0;
		}

	}
	return 0;
}


void sendUserPosMsg(XnUserID id) {
	XnPoint3D com;
	sprintf(tmp, "/user/%d", id);
	lo_bundle bundle = lo_bundle_new(LO_TT_IMMEDIATE);
	lo_message msg = lo_message_new();

	userGenerator.GetCoM(id, com);

	if (!raw)
	{
		lo_message_add_float(msg, (float)((1280 - com.X) / 2560));
		lo_message_add_float(msg, (float)((1280 - com.Y) / 2560));
		lo_message_add_float(msg, (float)(com.Z * 7.8125 / 10000));
	}
	else
	{
		lo_message_add_float(msg,com.X);
		lo_message_add_float(msg,com.Y);
		lo_message_add_float(msg,com.Z);
	}

	lo_bundle_add_message(bundle, tmp, msg);
	lo_send_bundle(addr, bundle);
}

void sendOSC() {

	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i) {
		if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
			lo_bundle bundle = lo_bundle_new(LO_TT_IMMEDIATE);

			if (jointPos(aUsers[i], XN_SKEL_HEAD) == 0) {
				oscFunc(&bundle, "head");
			}

			if (jointPos(aUsers[i], XN_SKEL_TORSO) == 0) {
				oscFunc(&bundle, "torso");
			}


			lo_send_bundle(addr, bundle);
		}
		else {
			//Send user's center of mass
			sendUserPosMsg(aUsers[i]);
		}
	}
}


// Generate OSC message with default format
void genOscMsg(lo_bundle *bundle, char *name) {

	if (posConfidence >= 0.5f)
	{

		lo_message msg = lo_message_new();		
		
		lo_message_add_string(msg, name);

		lo_message_add_int32(msg, -1);

	
		for(int i=0;i<3;i++)
		{
			lo_message_add_float(msg,jointCoords[i]);
		}
	
		if(sendOrient&& orientConfidence >=0.5f)
		{
			for(int i=0;i<3;i++)	
			{
				lo_message_add_float(msg, eulerAngles[i]);
			}

		}

		lo_bundle_add_message(*bundle,"/client",msg);
	}

}


int usage(char *name) {
	printf("\nUsage: %s [OPTIONS]\n\
Example: %s -a 127.0.0.1 -p 7110 -d 3 -n 1 -mx 1 -my 1 -mz 1 -ox 0 -oy 0 -oz 0\n\
\n\
(The above example corresponds to the defaults)\n\
\n\
Options:\n\
  -a <addr>\t Address to send OSC packets to (default: localhost).\n\
  -p <port>\t Port to send OSC packets to (default: 7110).\n\
  -w\t\t Activate depth view window.\n\
  -f\t\t Activate noise filter to reduce jerkyness.\n\
  -xr\t\tOutput raw kinect data\n\
  -xt\t\tOutput joint orientation data\n\
  -r\t\t Reverse image (disable mirror mode).\n\
  -s <file>\t Save to file (only .oni supported at the moment).\n\
  -h\t\t Show help.\n\n\
For a more detailed explanation of options consult the README file.\n\n",
		   name, name);
	exit(1);
}

void checkRetVal(XnStatus nRetVal) {
	if (nRetVal != XN_STATUS_OK) {
		printf("There was a problem initializing kinect... Make sure you have \
connected both usb and power cables and that the driver and OpenNI framework \
are correctly installed.\n\n");
		exit(1);
	}
}

void terminate(int ignored) {
	context.Shutdown();
	lo_address_free(addr);
	if (preview)
		glutDestroyWindow(window);
	exit(0);
}

void main_loop() {
	// Read next available data
	context.WaitAnyUpdateAll();
	// Process the data
	depth.GetMetaData(depthMD);
	sendOSC();
	if (preview)
		draw();
}


int main(int argc, char **argv) {
	printf("Initializing...\n");
	unsigned int arg = 1,
				 require_argument = 0,
				 port_argument = 0;
	XnMapOutputMode mapMode;
	XnStatus nRetVal = XN_STATUS_OK;
	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks, hHandsCallbacks, hGestureCallbacks;
	xn::Recorder recorder;

	context.Init();

	while ((arg < argc) && (argv[arg][0] == '-')) {
		switch (argv[arg][1]) {
			case 'a':
			case 'p':
				require_argument = 1;
				break;
			default:
				require_argument = 0;
				break;
		}

		if ( require_argument && arg+1 >= argc ) {
			printf("The option %s require an argument.\n", argv[arg]);
			usage(argv[0]);
		}

		switch (argv[arg][1]) {
		case 'h':
			usage(argv[0]);
			break;
		case 'a': //Set ip address
			ADDRESS = argv[arg+1];
			break;
		case 'p': //Set port
			if(sscanf(argv[arg+1], "%d", &PORT) == EOF ) {
				printf("Bad port number given.\n");
				usage(argv[0]);
			}
			port_argument = arg+1;
			PORT = argv[arg+1];
			break;
		case 'w':
			preview = true;
			break;
		case 's':
			checkRetVal(recorder.Create(context));
			checkRetVal(recorder.SetDestination(XN_RECORD_MEDIUM_FILE, argv[arg+1]));
			record = true;
			arg++;
			break;
		case 'f':
			filter = true;
			break;

		case 'r':
			mirrorMode = false;
			break;
        case 'x': //Set multipliers
			switch(argv[arg][2]) {
			case 'r': // turn on raw mode
				raw = true;
				break;
            case 't': // send joint orientations
				sendOrient = true;
				break;
			case 'd': // turn on default options
				raw = true;
				preview = true;
				sendOrient = true;
				mirrorMode = false;
				break;
			default:
				printf("Bad option given.\n");
				usage(argv[0]);
			}
			break;
		default:
			printf("Unrecognized option.\n");
			usage(argv[0]);
		}
		if ( require_argument )
			arg += 2;
		else
			arg ++;
	}

	if (oscFunc == NULL)
		oscFunc = genOscMsg;

	checkRetVal(depth.Create(context));


	nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
	if (nRetVal != XN_STATUS_OK)
		nRetVal = userGenerator.Create(context);

	checkRetVal(userGenerator.RegisterUserCallbacks(new_user, lost_user, NULL, hUserCallbacks));
	checkRetVal(userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(calibration_started, calibration_ended, NULL, hCalibrationCallbacks));
	checkRetVal(userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL));
		
	if (filter)
		userGenerator.GetSkeletonCap().SetSmoothing(0.8);


	xnSetMirror(depth, !mirrorMode);

	addr = lo_address_new(ADDRESS, PORT);
	signal(SIGTERM, terminate);
	signal(SIGINT, terminate);

	printf("Configured to send OSC messages to %s:%s\n", ADDRESS, PORT);

	printf("OSC Message format: ");
	

	printf("Default OSCeleton format\n");

	printf("Initialized Kinect, looking for users...\n\n");
	context.StartGeneratingAll();


	if (preview) {
		init_window(argc, argv, 640, 480, 500, 0, main_loop);
		glutMainLoop();
	}
	else {
		while(true)
			main_loop();
	}

	terminate(0);
}

