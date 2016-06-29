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
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.1 Rev1 on May 17th 2016

#include <iostream>
#include <controlplugin.h>
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


// --------------------------------------------------------------------------------------
// simExtBubble_create
double pParam=2, iParam=0, dParam=0, vParam=-2, cumul=0, lastE=0, pAlphaE=0, pBetaE=0, psp2=0, psp1=0, prevEuler=0;
float targetPos[3],pos[3],l[3],vx[3],vy[3],sp[3],euler[3],m[12],temp[3];
double e,pv,thrust,alphaE,betaE,rotCorr,alphaCorr,betaCorr,retvals[4];
int stack,d,targetObj;
void PID_callback(SScriptCallBack* cb) {
	stack = cb->stackID;
	simGetStackInt32Value(stack,&targetObj);
	simPopStackItem(stack,1);
	simGetStackInt32Value(stack,&d);
	simPopStackItem(stack,1);
    simGetObjectPosition(targetObj,-1,targetPos);
    simGetObjectPosition(d,-1,pos);
    simGetObjectVelocity(d,l,temp);
    e=(targetPos[2]-pos[2]);
    cumul=cumul+e;
    pv=pParam*e;
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[2]*vParam;
    lastE=e;
    // Horizontal control: 
    simGetObjectPosition(targetObj,d,sp);
    simGetObjectMatrix(d,-1,m);
    vx[0]=1;vx[1]=0;vx[2]=0; //ew
    simTransformVector(m,vx);
    vy[0]=0;vy[1]=1;vy[2]=0; //ew
    simTransformVector(m,vy);
    alphaE=(vy[2]-m[11]);
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE);
    betaE=(vx[2]-m[11]);
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE);
    pAlphaE=alphaE;
    pBetaE=betaE;
    alphaCorr=alphaCorr+sp[1]*0.005+1*(sp[1]-psp2);
    betaCorr=betaCorr-sp[0]*0.005-1*(sp[0]-psp1);
    psp2=sp[1];
    psp1=sp[0];
    
    // Rotational control:
    simGetObjectOrientation(d,-1,euler);
    rotCorr=euler[2]*0.1+2*(euler[2]-prevEuler);
    prevEuler=euler[2];
    retvals[0]=thrust*(1-alphaCorr+betaCorr+rotCorr);
	retvals[1]=thrust*(1-alphaCorr-betaCorr-rotCorr);
	retvals[2]=thrust*(1+alphaCorr-betaCorr+rotCorr);
	retvals[3]=thrust*(1+alphaCorr+betaCorr-rotCorr);
	simPushDoubleTableOntoStack(stack,retvals,4);
}


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
	simRegisterScriptCallbackFunction("simExtPidControl@v_repExtQuadControlPlugin","table_4 controlPluginHandle=(number self,number target)",PID_callback);


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
    	pParam=2, iParam=0, dParam=0, vParam=-2, cumul=0, lastE=0, pAlphaE=0, pBetaE=0, psp2=0, psp1=0, prevEuler=0;
    	//reset PID values
        //previousStopSimulationRequestCounter=-1;
    }

	return retVal;
}
