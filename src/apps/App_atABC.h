/**
@file
Application for communicating with OP320A&S SNM over RS232/485 (via RTU protocol).
*/
/*
  Application_SNM.h - Arduino library for communicating with OP320A&S SNM
  over RS232/485 (via RTU protocol).

  Library:: chickyPig_OP320_V1

  This version is

  Copyright:: 2021 nguyentrinhtuan1996@gmail.com
*/

#ifndef _Application_atABC_
#define _Application_atABC_
/* _____PROJECT INCLUDES____________________________________________________ */
#include "App.h"
#include "../services/Service_atXYZ.h"
/* _____DEFINETIONS__________________________________________________________ */

/* _____GLOBAL VARIABLES_____________________________________________________ */

///////////////////////////////////////////////Testing part//
/* _____GLOBAL FUNCTION______________________________________________________ */

/* _____CLASS DEFINETION_____________________________________________________ */
/**
 * This Application class is the application to manage the 
 */
class App_ABC : public Application
{
public:
  	App_ABC();
 	~App_ABC();
  	static void  App_ABC_Pend();
	static void  App_ABC_Start();
	static void  App_ABC_Restart();
	static void  App_ABC_Execute();
	static void  App_ABC_Suspend();
	static void  App_ABC_Resume();	  
	static void  App_ABC_End();
protected:
private:
} atABC ;
/**
 * This function will be automatical called when a object is created by this class
 */
App_ABC::App_ABC(/* args */)
{
  	_Pend_User 	     = *App_ABC_Pend;
	_Start_User 	 = *App_ABC_Start;
	_Restart_User 	 = *App_ABC_Restart;
	_Execute_User 	 = *App_ABC_Execute;
	_Suspend_User	 = *App_ABC_Suspend;
	_Resume_User	 = *App_ABC_Resume;
	_End_User	     = *App_ABC_End;

	// change the ID of application
	ID_Application = 1;
	// change the application name
	Name_Application = (char*)"ABC Application";
	// change the ID of SNM
}
/**
 * This function will be automatical called when the object of class is delete
 */
App_ABC::~App_ABC()
{
	
}
/**
 * Pend to start is the fisrt task of this application it will do prerequisite condition. In the debig mode, task will send information of application to terminal to start the application.
 */
void  App_ABC::App_ABC_Pend()
{
    atXYZ.Debug();
}
/**
 * This start function will init some critical function 
 */
void  App_ABC::App_ABC_Start()
{
	// init atXYZ Service in the fist running time
	atXYZ.Run_Service();
}  
/**
 * Restart function of SNM  app
 */
void  App_ABC::App_ABC_Restart()
{

}
/**
 * Execute fuction of SNM app
 */
void  App_ABC::App_ABC_Execute()
{	
	atXYZ.Run_Service();
    if(atABC.User_Mode == APP_USER_MODE_DEBUG)
    {
		
    }   
}
void  App_ABC::App_ABC_Suspend(){}
void  App_ABC::App_ABC_Resume(){}	  
void  App_ABC::App_ABC_End(){}

#endif