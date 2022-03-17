#include <SPI.h>
#include <Wire.h>
#include "../src/apps/App_INVT.h"
#include "../src/apps/App_Modbus.h"

//

// #define trong file FreeRTOSConfig.h
// #define configCHECK_FOR_STACK_OVERFLOW 1
// SemaphoreHandle_t modbusMutex;

TaskHandle_t Task_Modbus;  

void Modbus_Application(void *parameter)
{
  while (1)
  {
    atModbus.Run_Application(APP_RUN_MODE_AUTO);
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }
}


TaskHandle_t Task_INVT;
void INVT_Application(void *parameter)
{
  while(1)
  {
    // xSemaphoreTake( modbusMutex, portMAX_DELAY );
    INVT.Run_Application(APP_RUN_MODE_AUTO);
    // xSemaphoreGive( modbusMutex );
    vTaskDelay(2000/ portTICK_PERIOD_MS);
  }
}
void setup() {

    // delay(1000);

    // atModbus.Debug();
    INVT.Debug();


  {// create a task to run Modbus application             on core 1
    xTaskCreatePinnedToCore(
      Modbus_Application,  //task function
      "Modbus_Application",  // name task
      50000,  //stack size of task
      NULL,
      1,
      &Task_Modbus,
      1
    );
  };
    {// create a task to run INVT application                on core 1
      xTaskCreatePinnedToCore(
      INVT_Application,  //task function
      "INVT_GD200A_Application",  // name task
      50000,  //stack size of task
      NULL, 
      2,
      &Task_INVT,
      1
      );
    }
}

// void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
//  {
//      printf("\r\nStack Overflow hook, source: %s", pcTaskName);
//  }

void loop() {}

