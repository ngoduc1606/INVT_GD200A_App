 /**
@file
Application for communicating with OP320A&S SNM over RS232/485 (via RTU protocol).
*/
/*
  Library:: 

  This version is

  Copyright:: 2021 nguyentrinhtuan1996@gmail.com
*/

#ifndef _Application_INVT_
#define _Application_INVT_
/* _____PROJECT INCLUDES____________________________________________________ */
#include "App.h"
#include "../lib/INVT_GD200A.h"
#include <ModbusRTU.h>

/* _____DEFINETIONS__________________________________________________________ */

/* _____GLOBAL VARIABLES_____________________________________________________ */

/* _____GLOBAL FUNCTION______________________________________________________ */

/* _____CLASS DEFINETION_____________________________________________________ */
/**
 * This Application class is the application to manage the 
 */
class App_INVT : public Application, public INVT_GD200A
{
public:
  	App_INVT();
 	~App_INVT();
   
protected:
	static void  App_INVT_Pend();
	static void  App_INVT_Start();
	static void  App_INVT_Restart();
	static void  App_INVT_Execute();
	static void  App_INVT_Suspend();
	static void  App_INVT_Resume();	  
	static void  App_INVT_End();
    
private: 
} INVT ;

/**
 * This function will be automatical called when a object is created by this class
 */
App_INVT::App_INVT(/* args */)
{
  	_Pend_User 	     = *App_INVT_Pend;
	_Start_User 	 = *App_INVT_Start;
	_Restart_User 	 = *App_INVT_Restart;
	_Execute_User 	 = *App_INVT_Execute;
	_Suspend_User	 = *App_INVT_Suspend;
	_Resume_User	 = *App_INVT_Resume;
	_End_User	     = *App_INVT_End;

	// change the ID of application
	ID_Application = 2;
	// change the application name
	Name_Application = (char*)"INVT_Application";
	// change the ID of INVT
    ID_Modbus = 3;

}
/**
 * This function will be automatical called when the object of class is delete
 */
App_INVT::~App_INVT()
{
	
}
/**
 * Pend to start is the fisrt task of this application it will do prerequisite condition. In the debig mode, task will send information of application to terminal to start the application.
 */
void  App_INVT::App_INVT_Pend()
{

}
/**
 * This start function will init some critical function 
 */
void  App_INVT::App_INVT_Start()
{
//  INVT.Reset_Fault();
//  uint8_t Frequency_Limit_D = 50;                         //0.01*Hz
//  uint8_t Frequency_Limit_R = 50;                         //0.01*Hz
//  INVT.Set_Frequency_Limit(Frequency_Limit_D*100,Frequency_Limit_R*100);
}  
/**
 * Restart function of INVT  app
 */
void  App_INVT::App_INVT_Restart()
{

}
/**
 * Execute fuction of INVT app
 */
void  App_INVT::App_INVT_Execute()
{	
    
   
    
  
    // Fault INVT
    Serial.printf_P("        fault INVT : ");
    Serial.printf_P(INVT.Fault_Char());

    if (INVT.Fault_Code() == FAULT10)
    {
    INVT.Reset_Fault();
        { 
            Serial.printf_P("    reset fault : ");
        }
    }
    delay(500);
    uint8_t Frequency = 20;                         //0.01*Hz
    INVT.Set_Frequency(Frequency*100);
    Serial.printf_P("    set frequency : %d ", Frequency);
    delay(500);
    INVT.Run(FORWARD);
    delay(500);
    // Data form inverter
    INVT.Get_Data_From_INVT();
    Serial.printf_P("        Operating frequency: %.1f (Hz)\n",INVT.Frequency_Operating());
    Serial.printf_P("        Setting frequency: %.1f (Hz)\n",INVT.Frequency_Setting());
    Serial.printf_P("        Operating speed: %.1f (RPM)\n",INVT.Speed_Operating());
    Serial.printf_P("        Bus voltage: %.1f (V)\n",INVT.Voltage_Bus());
    Serial.printf_P("        Output voltage: %.1f (V)\n",INVT.Voltage_Output());
    Serial.printf_P("        Output currentL %.1f (A)\n", INVT.Current_Output());
    Serial.printf_P("        Output Power: %.1f (%%)\n", INVT.Power_Output());
    Serial.printf_P("        Output torque: %.1f (N.m)\n",INVT.Torque_Output());
}

void  App_INVT::App_INVT_Suspend(){}
void  App_INVT::App_INVT_Resume(){}	  
void  App_INVT::App_INVT_End(){}

#endif