/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// This is the starter code for the RTOS Homework designed for
// CSE/ECE 474 Summer 19.
//
// Please read through the entire file and make sure
// you understand your tasks, the structure of the code,
// and the hints you are given (although you are not
// required to follow them)
//
// The only requirement is to use the three tasks specified
// in the main function. Feel free to add/delete the code we
// provided, though it is recommended to follow the hints.
//
// Here's the documentation page for FreeRTOS,
// you may want to look at it when doing this assignment.
// https://www.freertos.org/FreeRTOS-quick-start-guide.html
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
//#include "myHeader.h"

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

// After skimming through the entire file,
// you might want to add some FSM states here.
// FSM state definition
const int OFF = 0;
const int GO = 1;
const int STOP = 2;
const int WARN = 3;

int state = 0;
int next_state = 0;


// Hint:
//
// The following global variables are flags you may want to use
int pedestrian_status = 0;
int onoff_status = 0;


// Below are function prototypes for our RTOS task.
// You should not change the function declerations.
// However, you are allowed to add more helper functions
// as needed.

// Initialize the GPIO Port for traffic light LEDs and user buttons
void GPIO_Init(void);

// Task function that checks the state of the pedestrian button.
// Keeps track of how many seconds the pedestrian button has been pressed.
// Once the user has pressed the pedestrian button for 2 seconds,
// it will set the global flag indicating the pedestrian button has been
// pressed
void Pedestrian(void *p);

// Task function that checks the state of the onoff button.
// Keeps track of how many seconds the onoff button has been pressed.
// Once the user has pressed the onoff button for 2 seconds,
// it will set the global flag indicating the onoff button has been
// pressed
void StartStop(void *p);

// Task function that represents your Finite State Machine.
// Keeps track of how many seconds the LED has been lighting on.
// It will update the state of the LED every 5 seconds or once there's
// button input
void Control(void *p);

void LightLED();

// Helper function that represents your FSM.
// You are allowed to change this function decleration.
//
// Handles the traffic light state transition.
void FSM(void);

/*
// Do not modify this function.
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {
  // This function can not return, so loop forever.  Interrupts are disabled
  // on entry to this function, so no processor interrupts will interrupt
  // this loop.
  while (1) {}
}
*/

// Initialize FreeRTOS and start the tasks.
int main_2(void) {
  // STEP 1
  //
  // This is the beginning of the main function,
  // Initialize your system by setting up all the GPIO pins 
  // you'll be using
  GPIO_Init();



  // STEP 2
  //
  // The code below creates three tasks.
  // Your task here is to assign priorities to the tasks.
  //
  // Think about which task(s) should be given the highest
  // priority and which should be given lower, it is possible to assign
  // the same priority to all the tasks though.
  //
  // Priorities are in range: [0, configMAX_PRIORITIES - 1], where
  // configMAX_PRIORITIES - 1 corresponds to the highest priority and
  // 0 corresponds to the lowest priority.
  // You can find configMAX_PRIORITIES defined in the file called FreeRTOSConfig.h
  // under the freertos_demo directory.
  //
  // You should not create more tasks. However, you are allowed to add as many
  // helper functions as you want.
  // xTaskCreate(Function Name,
  //             Descriptive Task Name,
  //             Stack Depth,
  //             Task Function Parameter,
  //             Priority,
  //             Task Handle);
  xTaskCreate(StartStop, (const char *)"StartStopButton", 1024, NULL, 14, NULL);
  xTaskCreate(Pedestrian, (const char *)"PedestrianButton", 1024, NULL, 14, NULL);
  xTaskCreate(Control, (const char *)"Control FSM", 1024, NULL, 13, NULL);

  // Start the scheduler. This should not return.
  // The scheduler will do the scheduling and switch between
  // different tasks for you.
  // Refer to the lecture slides and RTOS documentation
  // page for more details about task scheduling and context switching.
  //
  // One important concept for context switching and task scheduling
  // is preemption. Think about what do preemptive and non-preemptive
  // means.
  //
  // Hint: Non-preemptive scheduling will allow other tasks to be scheduled
  // after the current task has entered the "waiting state". That is, in our context,
  // the current task will keep running until it reaches vTaskDelay 
  // (you'll see this in task functions).
  //
  // You can find in FreeRTOSConfig.h the setting of preemption
  // for the RTOS. Feel free to change this setting as needed.
  vTaskStartScheduler();

  // In case the scheduler returns for some reason, loop forever.
  while(1) {}
}

void GPIO_Init() {
  // STEP 3
  //
  // Initialize your GPIO Port for traffic light LEDs
  // and user buttons
  volatile unsigned long delay;
  RCGCGPIO = 0x20; //enable clock for port F
  delay = RCGCGPIO;
  GPIODEN = 0x1F;  //port F APB pin 0,1,2,3 and 4 are digitally enabled
  GPIODIR = 0x0E;  //port F APB pin 1,2,3 output. 0 and 4 input
  GPIOLOCK = 0x4C4F434B;//unlocks register 
  GPIOCR = 0x11;// this should allow pins 0 and 4 to be written to 
  GPIOPUR = 0x11; //pins 0 and 4 are now pull up resistor
  GPIODATA = 0;
  /*
  RCGCGPIO |= 0x11; //activate clock for port A & E
  delay = RCGCGPIO;

  // PortE 0 -> RED
  // PortE 1 -> YELLOW
  // PortE 1 -> GREEN
  GPIODIR_PORTE |= 0x7;
  GPIODEN_PORTE |= 0x7;

  // PortA 2 -> onoff button
  // PortA 3 -> Pedestrian button
  GPIODIR_PORTA &= ~0xC;
  GPIODEN_PORTA |= 0xC;
*/
}

void StartStop(void *p) {
  // Hint:
  //
  // Static variable will only be decleared once and
  // will retain its last assigned value until the entire
  // program exits
  static int curr_onoff_tick_time = 0;
  static int prev_onoff_tick_time = 0;

  // xTaskGetTickCount() will return the count of ticks
  // since the Task Scheduler started (i.e. your program starts executing).
  //
  // The tick rate (configTICK_RATE_HZ) is defined in FreeRTOSConfig.h,
  // the default rate is 1000 (1KHz), so one tick is equivalent to 1 ms
  //
  // It is similar to the timer we used before, but you'll
  // need to calculate the time elapsed by taking the difference
  // between curr_tick and prev_tick.
  // curr_onoff_tick_time = xTaskGetTickCount();

  // STEP 4
  //
  // Complete the implementation of this task function.
  // It checks whether the StartStop button has been
  // pressed for 2 seconds and needs to set the global flags accordingly
  //
  // Task function should never return so is typically
  // implemented as an infinite loop
  //
  // The vTaskDelay(n) will cause the task to enter Blocked state
  // for n system clock ticks. i.e. the task is unblocked
  // after n systicks and will enter Ready State to be arranged for
  // running
  while (1) {
    curr_onoff_tick_time = xTaskGetTickCount();

    // Check whether the user switch is pressed.
    //
    // If the switch has been pressed for 2 or more seconds,
    // set the global flag to 1 and set the prev_tick to be equal
    // to curr_tick (clear the timer). Otherwise clear the global flag.
    //
    // If the user switch is not pressed, clear the global flag and
    // set the prev_tick to curr_tick.
    if ((GPIODATA & 0x01) == 0) { // replace this condition checking with something else
//      if (curr_onoff_tick_time - prev_onoff_tick_time >= 2000) {
        // Set the global flag
        // Clear the timer
        onoff_status = 1;
//        prev_onoff_tick_time = curr_onoff_tick_time;
//      }
//      else {
        // Clear the global flag
//         onoff_status = 0;
//      }
    } else {
      // Clear the global flag
      // Clear the timer
      onoff_status = 0;
//      prev_onoff_tick_time = curr_onoff_tick_time;
    }

    vTaskDelay(1);  
  }
}

void Pedestrian(void *p) {
  static int curr_ped_tick_time = 0;
  static int prev_ped_tick_time = 0;

  // STEP 5
  //
  // Complete the implementation of this task function.
  // It checks whether the pedestrian button has been pressed for
  // 2 seconds and update the global flag accordingly.
  //
  // Implement this based on the StartStop function.

  while (1) {
    curr_ped_tick_time = xTaskGetTickCount();
    if ((GPIODATA & 0x10) == 0) {
//      if (curr_ped_tick_time - prev_ped_tick_time >= 2000) {
        pedestrian_status = 1;
//        prev_ped_tick_time = curr_ped_tick_time;
//      } else {
//        pedestrian_status = 0;
//      }
    } else {
      pedestrian_status = 0;
//      prev_ped_tick_time = curr_ped_tick_time;
    }
    vTaskDelay(1);
  }
}

void Control(void *p) {
  static int curr_led_tick_time = 0;
  static int prev_led_tick_time = 0;

  // STEP 6
  //
  // Complete the implementation of this task function.
  // It represents your Finite State Machine.
  while (1) {
    curr_led_tick_time = xTaskGetTickCount();

    // If the led has been lighting up for 5 or more seconds,
    // or if any of the button flag has been set, switch to the
    // corresponding next state and reset the led tick.
    if ((curr_led_tick_time - prev_led_tick_time >= 5000)){// || onoff_status || pedestrian_status) {
      prev_led_tick_time = curr_led_tick_time;
      FSM();
    }
      
    vTaskDelay(1);
  }
}

// turn on corresponding LED
void LightLED() {
  GPIODATA &= ~(GREEN | RED | BLUE);
  if (state == WARN)
    GPIODATA |= BLUE;
  else if (state == GO)
    GPIODATA |= GREEN;
  else if (state == STOP)
    GPIODATA |= RED;
  else if (state == OFF) 
    GPIODATA &= ~(GREEN | RED | BLUE);
}

void FSM(void) {
  // STEP 7
  //
  // Copy your previous FSM implementation into this function.
  // This function will be called by the control task function.
  GPIODATA &= ~(GREEN | RED | BLUE);

  if(state == OFF) {
    GPIODATA |= (GREEN | RED | BLUE);
    if(!onoff_status && !pedestrian_status) { //00
      next_state = OFF;
    } else if (!onoff_status && pedestrian_status) { //01
      next_state = OFF;
    } else if (onoff_status && !pedestrian_status) { //10
      next_state = STOP;
    } else if (onoff_status && pedestrian_status) { //11
      next_state = WARN;
    }
  } else if (state == STOP) {
    GPIODATA |= RED;
    if(!onoff_status && !pedestrian_status) { //00
      next_state = OFF;
    } else if (!onoff_status && pedestrian_status) { //01
      next_state = OFF;
    } else if (onoff_status && !pedestrian_status) { //10
      next_state = GO;
    } else if (onoff_status && pedestrian_status) { //11
      next_state = WARN;
    }
  } else if (state == GO) {
    GPIODATA |= GREEN;
    if(!onoff_status && !pedestrian_status) { //00
      next_state = OFF;
    } else if (!onoff_status && pedestrian_status) { //01
      next_state = OFF;
    } else if (onoff_status && !pedestrian_status) { //10
      next_state = STOP;
    } else if (onoff_status && pedestrian_status) { //11
      next_state = WARN;
    }
  } else if (state == WARN) {
    GPIODATA |= BLUE;
    if(!onoff_status && !pedestrian_status) { //00
      next_state = OFF;
    } else if (!onoff_status && pedestrian_status) { //01
      next_state = OFF;
    } else if (onoff_status && !pedestrian_status) { //10
      next_state = STOP;
    } else if (onoff_status && pedestrian_status) { //11
      next_state = WARN;
    }
  }
  /*
  if (state == GO) {
    if (onoff_status)
      next_state = OFF;
    else if (pedestrian_status)
      next_state = WARN;
    else
      next_state = STOP;
  } else if (state == STOP) {
    if (onoff_status)
      next_state = OFF;
    else
      next_state = GO;
  } else if (state == WARN) {
    if (onoff_status)
      next_state = OFF;
    else
      next_state = STOP;
  } else {
    if (onoff_status)
      next_state = STOP;
  }
  */
  state = next_state;
  //LightLED();
}
