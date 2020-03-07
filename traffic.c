#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tm4c123gh6pm.h>
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
#define RED 0x02 //0010
#define BLUE 0x04 //0100
#define GREEN 0x08 //1000
#define YELLOW 
#define RGB 0x0E
#define RCGCGPIO       (*((volatile uint32_t *)0x400FE608))//good not sure why the example uses a different address
#define GPIODEN        (*((volatile uint32_t *)0x4002551C))//good
#define GPIODIR        (*((volatile uint32_t *)0x40025400))//good
#define GPIODATA       (*((volatile uint32_t *)0x400253FC))//good
#define GPIOLOCK       (*((volatile uint32_t *)0x40025520))
#define GPIOCR         (*((volatile uint32_t *)0x40025524))    
#define GPIOPUR        (*((volatile uint32_t *)0x40025510))   
#define RED_LED_PortA      0x80 
#define mainLED_TOGGLE_RATE ( ( portTickType ) 5000 / portTICK_RATE_MS )  
xQueueHandle Global_Queue_Handle = 0;


void traffic_task(void *p) {
  uint8_t ui8CurButtonState;

  while(1) {
    ui8CurButtonState = ButtonsPoll(0, 0);
    
    if(!xQueueSend(Global_Queue_Handle, &ui8CurButtonState, 1000)) {
      printf("Failed to send to queue\n");
    }
    /*
    if((ui8CurButtonState & ALL_BUTTONS) == LEFT_BUTTON) {
      GPIODATA = RED; //port F APB pin 1 is outputing HIG
    } else if ((ui8CurButtonState & ALL_BUTTONS) == RIGHT_BUTTON) {
      GPIODATA = BLUE;
    } else if ((ui8CurButtonState & ALL_BUTTONS) == ALL_BUTTONS) {
      GPIODATA = GREEN;
    }
*/
  }
}
void traffic_task_rx(void *p) {
  uint8_t ui8CurButtonState;

  while(1) {
    xQueueReceive(Global_Queue_Handle, &ui8CurButtonState, 1000);
      
    if(ui8CurButtonState == LEFT_BUTTON) {
      GPIODATA = RED; 
    } else if (ui8CurButtonState == RIGHT_BUTTON) {
      GPIODATA = BLUE;
    } else if (ui8CurButtonState == ALL_BUTTONS) {
      GPIODATA = GREEN;
    }
  }
}




// Initializes the switch task.

uint32_t
TrafficInit(void)
{
  RCGCGPIO = 0x20; //enable clock for port F
  GPIODEN = 0x1F;  //port F APB pin 0,1,2,3 and 4 are digitally enabled
  GPIODIR = 0x0E;  //port F APB pin 1,2,3 output. 0 and 4 input
  //GPIOLOCK = 0x4C4F434B;//unlocks register 
  //GPIOCR = 0x11;// this should allow pins 0 and 4 to be written to 
  //GPIOPUR = 0x11; //pins 0 and 4 are now pull up resistor
  GPIODATA = 0;
    //
    //
    // Create the traffic task.
    //
    xTaskCreate(traffic_task,"tx_task", 1024, NULL, 1, NULL);
    xTaskCreate(traffic_task_rx,"rx_task", 1024, NULL, 1, NULL);
    
    return(0);
}




//rule: task must not exit, must be in a infinite loop.
void sender_task(void *p) {
  int i = 0;
  while(1) {
    printf("Send %i to receiver task\n", i);
    //(queue handle/name, item to queue(always a pointer), ticks willing to wait IF queue is full)
    //returns 0 if cant send 1 if sent
    if(!xQueueSend(Global_Queue_Handle, &i, 1000)) {
      printf("Failed to send to queue\n");
    }
    ++i;
    vTaskDelay(2000); //approx 1 unit = 1ms. puts this task to sleep
  }
}

//rule: task must not exit, must be in a infinite loop.
void receiver_task(void *p) {
  int rx_int = 0;
  while(1) {
    //(name of queue, where you want to copy data to(always a pointer), how long you want to wait to recieve an item if no item in queue)
    if(xQueueReceive(Global_Queue_Handle, &rx_int, 1000)) {
      printf("Received %i\n", rx_int);
    } else {
      printf("Failed to receive data from queue\n");
    }
  }
}

int traffic (void) {
  
  //                    (size of queue, size of one item)
  Global_Queue_Handle = xQueueCreate(2, sizeof(uint8_t));
  //creats tasks
  //         (function name, name of task, stack size, this gets passed into method, priority(lower number = lower priortiy),task handle(optinal))
  //xTaskCreate(sender_task  ,"tx_task", 1024, NULL, 1, NULL);
  //xTaskCreate(receiver_task,"rx_task", 1024, NULL, 1, NULL);
  TrafficInit();
  ButtonsInit();
  //start sceduler
  vTaskStartScheduler(); 
  
  return 0;
}

