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

#ifndef _Application_ModBus_
#define _Application_ModBus_
/* _____PROJECT INCLUDES____________________________________________________ */
#include "App.h"
#include <ModbusRTU.h>
#include <SoftwareSerial.h>
#include "../lib/INVT_GD200A.h"

/* _____DEFINETIONS__________________________________________________________ */

/* _____GLOBAL VARIABLES_____________________________________________________ */

#ifndef _SoftwareSerial_Modbus_1_
#define _SoftwareSerial_Modbus_1_
#define MODBUS_RX_PIN 27
#define MODBUS_TX_PIN 14
SoftwareSerial Serial_Modbus_1(MODBUS_RX_PIN, MODBUS_TX_PIN);
#endif

/* _____GLOBAL FUNCTION______________________________________________________ */
/* _____CLASS DEFINETION_____________________________________________________ */
/**
 * This Application class is the application to manage the 
 */
class App_Modbus : public Application
{
public:
  	App_Modbus();
 	~App_Modbus();
  	static void  App_Modbus_Pend();
	static void  App_Modbus_Start();
	static void  App_Modbus_Restart();
	static void  App_Modbus_Execute();
	static void  App_Modbus_Suspend();
	static void  App_Modbus_Resume();	  
	static void  App_Modbus_End();

	bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data);
	bool Transaction_Is_Done = 0;
	uint8_t Error = 0;
	void Wait_For_Done_Transaction();
protected:

    
private: 
} atModbus ;
/**
 * This function will be automatical called when a object is created by this class
 */
App_Modbus::App_Modbus(/* args */)
{
  	_Pend_User 	     = *App_Modbus_Pend;
	_Start_User 	 = *App_Modbus_Start;
	_Restart_User 	 = *App_Modbus_Restart;
	_Execute_User 	 = *App_Modbus_Execute;
	_Suspend_User	 = *App_Modbus_Suspend;
	_Resume_User	 = *App_Modbus_Resume;
	_End_User	     = *App_Modbus_End;

	// change the ID of application
	ID_Application = 1;
	// change the application name
	Name_Application = (char*)"Modbus_Application";
	// change the ID of SNM
}
/**
 * This function will be automatical called when the object of class is delete
 */
App_Modbus::~App_Modbus()
{
	
}
/**
 * Pend to start is the fisrt task of this application it will do prerequisite condition. In the debig mode, task will send information of application to terminal to start the application.
 */
void  App_Modbus::App_Modbus_Pend()
{
	
}
/**
 * This start function will init some critical function 
 */
void  App_Modbus::App_Modbus_Start()
{
	Serial_Modbus_1.begin(9600,SWSERIAL_8E1);
  	mb.begin(&Serial_Modbus_1);
	mb.master();
}  
/**
 * Restart function of SNM  app
 */
void  App_Modbus::App_Modbus_Restart()
{
	
}
/**
 * Execute fuction of SNM app
 */
void  App_Modbus::App_Modbus_Execute()
{
	mb.task();
	yield();
}
void  App_Modbus::App_Modbus_Suspend(){
 Serial.printf_P("123");
}
void  App_Modbus::App_Modbus_Resume(){}	  
void  App_Modbus::App_Modbus_End(){}


bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) 
{
	if (atModbus.User_Mode == APP_USER_MODE_DEBUG)
  		Serial.printf_P("	Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
	// the transcation is done and module is ready for another transaction
	atModbus.Transaction_Is_Done = 1;
	atModbus.Error = event;
  	return true;
}


void  App_Modbus::Wait_For_Done_Transaction()
{
	Transaction_Is_Done = 0;
	Error = 0;
	// wait for transaction is done
	while (!Transaction_Is_Done)
	{
		mb.task();
		yield();
	}
}

#endif