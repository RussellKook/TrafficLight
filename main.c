#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "switch_task.h"
#include "led_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sleep.h"

void my_task(void *p) {
  while(1) {
    printf("Hello World");
  }
}

int main (void) {
  
  //creats tasks
  //         (function name, name of task, stack size, this gets passed into method, priority(lower number = lower priortiy),task handle(optinal))
  xTaskCreate(my_task,"my_task", 1024, NULL, 1, NULL);
  
  //start sceduler
  vTaskStartScheduler(); 
  
  
  
  return 0;
}