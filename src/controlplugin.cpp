// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
//Plugin author:
//Matt Buckley,
//University of Southern California

#include <iostream>
#include <map>
#include <controlplugin.h>
#include "sim_module_variables.h"
#include "simulation_variables.h"
#include "position_external.h"
#include "mathconstants.h"
#include "stabilizer_types.h"
#include "replaced_methods.h"

extern "C" {
#include <ekf.h>
}
#ifdef _WIN32
	#ifdef QT_COMPIL
		#include <direct.h>
	#else
		#include <shlwapi.h>
		#pragma comment(lib, "Shlwapi.lib")
	#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
	#define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)


LIBRARY vrepLib;


/*
static control_t control;
static controllerState_t controllerState;
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
*/
static void ekf_flip() {
	struct ekf *ekf_temp = ekf_front;
	ekf_front = ekf_back;
	ekf_back = ekf_temp;
}
bool initialized = false; //probably unnecessary
int stack, targetObj;
float targetPos[3],temp[3],targetVel[3],targetAngVel[3],targetQuat[4],targetAccel[3],targetAngles[3];
float targetMatrix[12];
float v_last[3]; //we'll get a neater way to calculate acceleration once we have a dynamics model
double retvals[4];
double last_vicon = 0;
bool run_vicon = true;
bool positionExternalFresh = false;
bool new_quad = false;
quad_env_data quad_internals[max_quads];

int id_to_handle[max_quads];
std::map<int,int> handle_to_id;

#define vicon_sim_hz 100.0
#define vicon_sim_step 1.0/vicon_hz

long int current_tick = 0;
int total_quads = 0;
int ATTITUDE_UPDATE_RATE;
float ATTITUDE_UPDATE_DT;
/*
sensorsAcquire(&sensorData, tick);
stateEstimator(&state, &sensorData, tick);
commanderGetSetpoint(&setpoint, &state);
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      // Rate-controled YAW is moving YAW angle setpoint
      if (setpoint.mode.yaw == modeVelocity) {
         setpoint.attitude.yaw -= setpoint.attitudeRate.yaw/500.0;
        while (setpoint.attitude.yaw > 180.0)
          setpoint.attitude.yaw -= 360.0;
        while (setpoint.attitude.yaw < -180.0)
          setpoint.attitude.yaw += 360.0;
      }

      if (!setpoint.enablePosCtrl) {
        setpoint.attitudeRate.yaw = 0;
      }

      positionControllerMellinger(&control, &state, &setpoint, &controllerState, 0.01);

      trajectoryState_t trajectoryState;
      trajectoryGetState(&trajectoryState);

      if (!setpoint.enablePosCtrl) {
        control.thrust = setpoint.thrust;
      }

      m_roll = control.roll;
      m_pitch = control.pitch;
      m_yaw = control.yaw;

      if (control.thrust == 0
          || ( setpoint.enablePosCtrl &&
             ( !sensorData.valid
              || trajectoryState == TRAJECTORY_STATE_IDLE)))
      {
        control.thrust = 0;
        control.roll = 0;
        control.pitch = 0;
        control.yaw = 0;

        // attitudeControllerResetAllPID();
        positionControllerReset(&controllerState);
        trajectorySetState(TRAJECTORY_STATE_IDLE);
        setpoint.attitude.yaw = state.attitude.yaw;
      }
    }
    powerDistribution(&control);
    */

#define limitThrust(VAL) limitUint16(VAL)



//put this in a lib somewhere
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state) {// update setpoint
	trajectoryPoint_t goal;
	//trajectoryGetCurrentGoal(&goal); lol no
	setpoint->position.x = goal.x;
	setpoint->position.y = goal.y;
	setpoint->position.z = goal.z;
	setpoint->velocity.x = goal.velocity_x;
	setpoint->velocity.y = goal.velocity_y;
	setpoint->velocity.z = goal.velocity_z;
	setpoint->attitude.yaw = goal.yaw;
	setpoint->attitudeRate.roll = goal.omega.x;
	setpoint->attitudeRate.pitch = goal.omega.y;
	setpoint->attitudeRate.yaw = goal.omega.z;
	setpoint->mode.x = modeAbs;
	setpoint->mode.y = modeAbs;
	setpoint->mode.z = modeAbs;
	setpoint->mode.roll = modeDisable;
	setpoint->mode.pitch = modeDisable;
	setpoint->mode.yaw = modeAbs;
	setpoint->enablePosCtrl = true;
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick) {
    simGetObjectPosition(targetObj,-1,targetPos);
	simGetObjectQuaternion(targetObj,-1,targetQuat);
	simGetObjectOrientation(targetObj,-1,targetAngles);
    simGetObjectVelocity(targetObj,targetVel,targetAngVel);
    targetAngles[1] = -targetAngles[1];
    targetAngVel[1] = -targetAngVel[1];
    temp[0]=0;
    temp[1]=0;
    temp[2]=0;
	simBuildMatrixQ(temp,targetQuat,targetMatrix); //get rotation matrix
	simTransformVector(targetMatrix,targetAngVel);
	for (int i=0;i<3;++i) { //replace acceleration calculations here with something more elegant later
		targetAccel[i] = (targetVel[i]-v_last[i])/simGetSimulationTimeStep();
	}
	targetAccel[2] +=  9.81;
	simTransformVector(targetMatrix,targetAccel); //transform to current coordinate frame
    simGetObjectVelocity(targetObj,v_last,temp);
	sensors->valid = true;
	sensors->gyro.x = degrees(targetAngVel[0]);
	sensors->gyro.y = degrees(targetAngVel[1]);
	sensors->gyro.z = degrees(targetAngVel[2]);
	sensors->acc.x = targetAccel[0]/GRAV;
	sensors->acc.y = targetAccel[1]/GRAV;
	sensors->acc.z = targetAccel[2]/GRAV;
	//don't need to call positionExternalGetLastData when not looping vicon because data is preserved in the struct, I think
	if (run_vicon || new_quad) {
		positionExternalFresh = true;
		sensors->position.x = targetPos[0];
		sensors->position.y = targetPos[1];
		sensors->position.z = targetPos[2];
		/*sensors->quaternion.q0 = targetQuat[0];
		sensors->quaternion.q1 = targetQuat[1];
		sensors->quaternion.q2 = targetQuat[2];
		sensors->quaternion.q3 = targetQuat[3];*/
		struct vec ang = mkvec(targetAngles[0],targetAngles[1],targetAngles[2]);
		struct quat dir = rpy2quat(ang); //iguess
		sensors->quaternion.x = dir.x;
		sensors->quaternion.y = dir.y;
		sensors->quaternion.z = dir.z;
		sensors->quaternion.w = dir.w;
	}
}

void step_firmware(SScriptCallBack* cb) { //takes no args
	sim_rate_dt = simGetSimulationTimeStep();
	sim_rate_hz = (int)(1.0/sim_rate_dt+0.5); //rough round
	current_tick += sim_rate_hz;
	if (run_vicon || (current_tick>(last_vicon+vicon_sim_step))) { //basic vicon simulation
		last_vicon = time;
		run_vicon = true;
	}
	for (current_quad = 0; current_quad < total_quads; ++current_quad) {
		set_context(&quad_data[current_quad].context);
		//simulate sensors
		//call state estimation
		targetObj = id_to_handle[current_quad];
		sensorsAcquire(&quad_internals[current_quad].sensorData,current_tick);
		ATTITUDE_UPDATE_RATE = sim_rate_hz;
		ATTITUDE_UPDATE_DT = sim_rate_dt;
		stateEstimator(&state, &sensorData, current_tick);
		commanderGetSetpoint(&setpoint, &state);
		if (setpoint.mode.yaw == modeVelocity) {
	 		setpoint.attitude.yaw -= setpoint.attitudeRate.yaw/sim_rate_hz;
		while (setpoint.attitude.yaw > 180.0)
			setpoint.attitude.yaw -= 360.0;
		while (setpoint.attitude.yaw < -180.0)
			setpoint.attitude.yaw += 360.0;
		}
		positionControllerMellinger(&control, &state, &setpoint, &controllerState, sim_rate_dt);
		trajectoryState_t trajectoryState;
		trajectoryGetState(&trajectoryState);
		if (control.thrust == 0
		  || ( setpoint.enablePosCtrl &&
		     ( !sensorData.valid
		      || trajectoryState == TRAJECTORY_STATE_IDLE)))
		{
			control.thrust = 0;
			control.roll = 0;
			control.pitch = 0;
			control.yaw = 0;

			// attitudeControllerResetAllPID();
			positionControllerReset(&controllerState);
			trajectorySetState(TRAJECTORY_STATE_IDLE);
			setpoint.attitude.yaw = state.attitude.yaw;
		}
		save_context(&quad_data[current_quad].context);
	}
	run_vicon = false;
}

void register_quadrotor(SScriptCallBack* cb) {
	new_quad = true;
	int new_handle;
	simGetStackInt32Value(stack,&new_handle);
	simPopStackItem(stack,1);
	quad_internals[total_quads].ekf_initialized = false;
	quad_internals[total_quads].first_vicon = true;
	sensorsAcquire(&quad_internals[total_quads].sensorData,current_tick);

	//initialize other unique quad state stuffs

	id_to_handle[total_quads] = new_handle;
	handle_to_id[new_handle] = total_quads;
	new_quad = false;
	++total_quads;
}
/*

void ekf_callback(SScriptCallBack* cb) { //soon to be defunct
	stack = cb->stackID;
	simGetStackInt32Value(stack,&targetObj);
	simPopStackItem(stack,1);

	if (!initialized)  {
		//float init[] = {0, 0, 0, 1};
		//ekf_init(ekf_back, init, init, init);
		last_vicon_sim = simGetSimulationTime();
	    simGetObjectPosition(targetObj,-1,targetPos);
	    simGetObjectVelocity(targetObj,targetVel,temp);
	    simGetObjectQuaternion(targetObj,-1,targetQuat);

		simBuildMatrixQ(temp,targetQuat,targetMatrix);
		simTransformVector(targetMatrix,targetVel);

		ekf_init(ekf_back, targetPos, targetVel, targetQuat);
		ekf_init(ekf_front, targetPos, targetVel, targetQuat);
	    simGetObjectVelocity(targetObj,v_last,temp);
	    initialized = true;
	}
	else {
		//maths to get ekf-style data from sim
	    simGetObjectPosition(targetObj,-1,targetPos);
	    simGetObjectVelocity(targetObj,targetVel,targetAngVel);
	    temp[0]=0;
	    temp[1]=0;
	    temp[2]=0;
	    simGetObjectQuaternion(targetObj,-1,targetQuat);
		simBuildMatrixQ(temp,targetQuat,targetMatrix); //get rotation matrix
		simTransformVector(targetMatrix,targetAngVel);
		for (int i=0;i<3;++i) {
			targetAccel[i] = (targetVel[i]-v_last[i])/simGetSimulationTimeStep();
		}
		targetAccel[2] +=  9.81;
		simTransformVector(targetMatrix,targetAccel); //transform to current coordinate frame
	    simGetObjectVelocity(targetObj,v_last,temp);

	    //run ekf
		ekf_imu(ekf_back, ekf_front, targetAccel, targetAngVel, simGetSimulationTimeStep());
		ekf_flip();

		double time=simGetSimulationTime();
		if (time>(last_vicon+vicon_sim_step)) { //basic vicon simulation
			last_vicon = time;
			ekf_vicon(ekf_back, ekf_front, targetPos, targetQuat);
			ekf_flip();
		}
	    for (int i=0;i<4;++i) {
	    	retvals[i] = 8;
	    }
	    //std::cout << "TARGET VELOCITY:" << std::endl;
	    //std::cout << targetVel[0] << ", " << targetVel[1] << ", " << targetVel[2] << std::endl;
	    //std::cout << "TARGET ACCEL:" << std::endl;
	    //std::cout << targetAccel[0] << ", " << targetAccel[1] << ", " << targetAccel[2] << std::endl;
	    //std::cout << "EKF POSITION:" << std::endl;
	    //std::cout << ekf_back->pos.x << ", " << ekf_back->pos.y << ", " << ekf_back->pos.z << std::endl;
	    //std::cout << "TRUE POSITION:" << std::endl;
	    //std::cout << targetPos[0] << ", " << targetPos[1] << ", " << targetPos[2] << std::endl;
		simPushDoubleTableOntoStack(stack,retvals,4);
	}
}
*/
/*
void ekf_initialize(SScriptCallBack* cb) {
	//initialized = false;
	
	stack = cb->stackID;
	simGetStackInt32Value(stack,&targetObj);
	//float init[] = {0, 0, 0, 1};
	//ekf_init(ekf_back, init, init, init);
    simGetObjectPosition(targetObj,-1,targetPos);
    simGetObjectVelocity(targetObj,targetVel,temp);
	for (int i=0;i<3;++i) {
		v_last[i] = targetVel[i];
	}
    simGetObjectQuaternion(targetObj,-1,targetQuat);
	ekf_init(ekf_back, targetPos, targetVel, targetQuat);
	ekf_init(ekf_front, targetPos, targetVel, targetQuat);
	
}*/

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.
	// Dynamically load and bind V-REP functions:
	char curDirAndFile[1024];
#ifdef _WIN32
	#ifdef QT_COMPIL
		_getcwd(curDirAndFile, sizeof(curDirAndFile));
	#else
		GetModuleFileName(NULL,curDirAndFile,1023);
		PathRemoveFileSpec(curDirAndFile);
	#endif
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'ControlPlugin' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start 'ControlPlugin' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	// Check the V-REP version:
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
	{
		std::cout << "Sorry, your V-REP copy is somewhat old, V-REP 3.2.0 or higher is required. Cannot start 'BubbleRob' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	// Register 4 new Lua commands:
	//simRegisterScriptCallbackFunction("simExtPidControl@v_repExtQuadControlPlugin","table_4 controlPluginHandle=(number self,number target)",PID_callback);
	simRegisterScriptCallbackFunction("simExtCallEkf@v_repExtQuadControlPlugin","controlPluginHandle=(number self)",ekf_callback);
	simRegisterScriptCallbackFunction("simExtStartEkf@v_repExtQuadControlPlugin","controlPluginHandle=(number self)",ekf_initialize);

	return(8); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
	// version 1 was for V-REP versions before V-REP 2.5.12
	// version 2 was for V-REP versions before V-REP 2.6.0
	// version 5 was for V-REP versions before V-REP 3.1.0 
	// version 6 is for V-REP versions after V-REP 3.1.3
	// version 7 is for V-REP versions after V-REP 3.2.0 (completely rewritten)
	// version 8 is for V-REP versions after V-REP 3.3.0 (using stacks for data exchange with scripts)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
	unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	// This function should not generate any error messages:

	void* retVal=NULL;

    if(message == sim_message_eventcallback_simulationabouttostart)
    {
    	//pParam=2, iParam=0, dParam=0, vParam=-2, cumul=0, lastE=0, pAlphaE=0, pBetaE=0, psp2=0, psp1=0, prevEuler=0;
    	
    	//reset PID values
        //previousStopSimulationRequestCounter=-1;
    }

	return retVal;
}
