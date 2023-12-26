/* Project: ECE-GY 6483 Embedded Challenge
*  Group 1
*  Member: Ye Ye(yy4820), Ruanxiao He(rh3895), Samin Ghasemi(sg7884)
*  main() at line:
*/


// Dependencies
#include <mbed.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <fstream>
#include <string>
#include <iostream>
#include "drivers/LCD_DISCO_F429ZI.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include "stm32f4xx_hal.h"
#include <chrono>

#define WINDOW_SIZE 15  // Example window size, adjust as needed

#define RECORDING_TIME 20

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Sqond configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000
#define OUT_X_L 0x28
#define ang2DegScalingFactor (8.75f * 0.017453292519943295769236907684886f / 1000.0f)
#define FILTER_COEFFICIENT 0.9f // Adjust this value as needed
#define Radius 0.85

#define MOSI_PIN PF_9
#define MISO_PIN PF_8
#define SCLK_PIN PF_7
#define CS_PIN  PC_1
#define SPI_FLAG 1
#define DATA_READY_FLAG 2

// define State for state machine
#define STATUS_IDLE 0
#define STATUS_RECORDING 1
#define STATUS_FINISH 2

// global variables
LCD_DISCO_F429ZI lcd;
int CurrentStatus; //current status
Thread drawingThread; // a specialized thread for drawing
Semaphore data_prepare_semaphore(1); // a specialized semaphore for data preparing

// data recording
Ticker timer;
Timeout t_out;
double totalDist = 0;
float window_gx[WINDOW_SIZE] = {0};
float window_gy[WINDOW_SIZE] = {0};
float window_gz[WINDOW_SIZE] = {0};
int window_index = 0;
float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;
float gX_ref = 0.0f, gY_ref = 0.0f, gZ_ref = 0.0f;

const int MAX_DATA_POINTS = 1000;
float linearVelocity[MAX_DATA_POINTS];
int dataIndex = 0;
float currentVelocity = 0.f;

// Fucntion Prototype
void spi_cb(int event); // Callback function for SPI events
void btn_cb();      // Callback function for button press
void timeout_cb();  // Callback function for timeouts
void timer_cb();  // Callback function for the timer
void data_cb(); // Callback function for data ready event
double* getGyroDim(int32_t xDimOut_L);
double calculateDist3Dim(double gyroDimDegRes[]);
// UI drawing functions
void drawThreadFunction();  // Function for the drawing thread to update the UI
void IDLE_Screen();         // Function to display the idle screen
void Recording_Screen();    // Function to display the recording screen
void FINISH_Screen();       // Function to display the finish screen

//======================================================================
EventFlags flags;           // Flags used for signaling events in the program
uint8_t write_buf[32];      // Buffer to store data to be written via SPI
uint8_t read_buf[32];       // Buffer to store data read from SPI
InterruptIn btn(BUTTON1, PullDown);  // Setup of the interrupt for the button
volatile int flag = 0;      // Variable to store flags for SPI communication (mosi, miso, sclk)
double result[3];           // Array to store results, possibly from gyro or other sensors

// spi initialization:
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN, CS_PIN, use_gpio_ssel);

int main()
 {
  //interrupt initialization:
  InterruptIn int2(PA_2, PullDown);
  int2.rise(&data_cb);

  btn.rise(&btn_cb);

  //ticker
  timer.attach(&timer_cb, 500ms);
  
  //spi format and frequency:
  spi.format(8, 3);
  spi.frequency(1'000'000);

  // Write to control registers --> spi transfer
  write_buf[0] = CTRL_REG1;
  write_buf[1] = CTRL_REG1_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb,SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  write_buf[0] = CTRL_REG4;
  write_buf[1] = CTRL_REG4_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb,SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  write_buf[0] = CTRL_REG3;
  write_buf[1] = CTRL_REG3_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb,SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  write_buf[1] = 0xFF;

  //(polling for\setting) data ready flag
  if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) 
  {
    flags.set(DATA_READY_FLAG);
  }

  // start drawing the screen
  drawingThread.start(drawThreadFunction);

  while (1)
  {
    // State machine for flow control of the project
    switch (CurrentStatus)
    {
      case STATUS_IDLE:
      {
        totalDist = 0;
        currentVelocity = 0.f;
        dataIndex = 0;
        break;
      }
      case STATUS_RECORDING:
      {
        double *gyroCurrDimData = getGyroDim(OUT_X_L);
        totalDist += (calculateDist3Dim(gyroCurrDimData) * 1000);
        printf("\n\t\t\Total Distance Travelled So Far:%f\t\n", totalDist);
        break;
      }
      // Wait for button press to start matching
      case STATUS_FINISH:
      {
        break;
      }
      default:
      {
        CurrentStatus = STATUS_IDLE;
        totalDist = 0;
        break;
      }
    }
    // the main thread realease the semaphore after preparing the data
    data_prepare_semaphore.release();
  }
}

// Timer Interrupt callback function
void timeout_cb() {
  CurrentStatus = STATUS_FINISH;
}

// Button Interrupt callback funtion
void btn_cb() {
  switch (CurrentStatus) {
    // During recording, update distance based on gyroscope data
    case STATUS_IDLE:
    {
      CurrentStatus = STATUS_RECORDING;
      t_out.attach(timeout_cb, RECORDING_TIME);
      break;
    }
    case STATUS_RECORDING:
    {
      t_out.detach();
      CurrentStatus = STATUS_IDLE;
      break;
    }
    case STATUS_FINISH:
    {
      CurrentStatus = STATUS_IDLE;
      break;
    }
    default:
    {
       // Default case resets to idle state and clears total distance
      CurrentStatus = STATUS_IDLE;
      break;
    }
  }
}

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}
// Callback for handling timeout events
void timer_cb() {
    spi_cb(0); // Call the spi_cb with a default event value
}

// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

//========================Data Processing ==================================
double* getGyroDim(int32_t xDimOut_L)
{
    //Unfiltered data:
    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    flags.wait_all(DATA_READY_FLAG);
    write_buf[0] = xDimOut_L | 0x80 | 0x40;

    spi.transfer(write_buf, 7, read_buf, 7, spi_cb,SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // Process raw data
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    //Filter Begin:
    gx = ((float)raw_gx) * ang2DegScalingFactor;
    gy  = ((float)raw_gy) * ang2DegScalingFactor;
    gz  = ((float)raw_gz) * ang2DegScalingFactor;

    window_gx[window_index]  = gx;
    window_gy[window_index]  = gy;
    window_gz[window_index]  = gz;

    //Compute the moving average   
    float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) 
    {
        avg_gx += window_gx[i];
        avg_gy += window_gy[i];
        avg_gz += window_gz[i];
    }
    avg_gx /= WINDOW_SIZE;
    avg_gy /= WINDOW_SIZE;
    avg_gz /= WINDOW_SIZE;

    //Increment and wrap the window index
    float gx_linear = (avg_gx /360) * (2*3.142*Radius);  
    float gy_linear = (avg_gy /360) * (2*3.142*Radius) ;
    float gz_linear = (avg_gz /360) * (2*3.142*Radius) ;

    float forward_linear = sqrt(gx_linear * gx_linear + gy_linear * gy_linear + gz_linear * gz_linear);
    currentVelocity = forward_linear;
    
    if(dataIndex < MAX_DATA_POINTS) {
      linearVelocity[dataIndex++] = currentVelocity;
      printf("linearVelocity[%d] = %f\n", dataIndex - 1, linearVelocity[dataIndex - 1]);
    }
    float gx_length = gx_linear * 0.5;
    float gy_length = gy_linear * 0.5;
    float gz_length = gz_linear * 0.5;

    thread_sleep_for(500);

    result[0] = gx_length;
    result[1] = gy_length;
    result[2] = gz_length;

    window_index = (window_index + 1) % WINDOW_SIZE;

    return result;

    // window_index = (window_index + 1) % WINDOW_SIZE;
    
    // //Example 2: Apply Simple low-pass filter
    // filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
    // filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
    // filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

    // float gx_length = (filtered_gx /360)* (2*3.142*0.85) * 0.5;
    // float gy_length = (filtered_gy /360)* (2*3.142*0.85) * 0.5;
    // float gz_length = (filtered_gz /360)* (2*3.142*0.85) * 0.5;

    // //printf("Filtered LENGTH:-> \tgx: %f \t gy: %f \t gz: %f\n", gy_length, raw_gy, gz_length);
    // //printf("~~~~~~Filtered LENGTH:-> \tgx: %f \t gy: %f \t gz: %f\n", filtered_gx, filtered_gy, filtered_gz);
    // //printf("\nFiltered LENGTH:-> \tgx: %f \t gy: %f \t gz: %f\n", gx_length, gy_length, gz_length);
    // thread_sleep_for(500);

    // if((abs(gx_length*1000000)>=100) || (abs(gy_length*1000000)>=100) || (abs(gz_length*1000000)>=100))
    // {
    //     result[0]=gx_length;
    //     result[1]=gy_length;
    //     result[2]=gz_length;
    // }
    // else
    // {
    //    result[0]=0;
    //    result[1]=0;
    //    result[2]=0; 
    // }
    // return result;
}

double calculateDist3Dim(double gyroDimDegRes[])
{
  double xSq = (gyroDimDegRes[0] - gX_ref) * (gyroDimDegRes[0] - gX_ref);
  double ySq = (gyroDimDegRes[1] - gY_ref) * (gyroDimDegRes[1] - gY_ref);
  double zSq = (gyroDimDegRes[2] - gZ_ref) * (gyroDimDegRes[2] - gZ_ref);

  gX_ref = gyroDimDegRes[0];
  gY_ref = gyroDimDegRes[1];
  gZ_ref = gyroDimDegRes[2];

  return sqrt(xSq+ ySq + zSq);
}

// ===================== UI drawing functions ====================
void drawThreadFunction() {
  while (1) {
    // wait for main thread to release a semaphore,
    // to indicate a new sample is ready to be graphed
    data_prepare_semaphore.acquire();

    // update the screen
    switch (CurrentStatus) {
      case STATUS_IDLE:
      {
        IDLE_Screen();
        break;
      }
      // Get original data while LED1 is on (green)
      case STATUS_RECORDING:
      {
        Recording_Screen();
        break;
      }
      // Wait for button press to start matching
      case STATUS_FINISH:
      {
        FINISH_Screen();
        break;
      }
      default:
      {
        IDLE_Screen();
        break;
      }
    }
  }
}
void IDLE_Screen()
{
  lcd.Clear(LCD_COLOR_WHITE);
  lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"ECE 6483", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Final Project", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"The Need for Speed", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Group: 1", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Press Button to Start", CENTER_MODE);

  lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"Ye Ye(yy4820)", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"Ruanxiao He(rh3895)", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"Samin Ghasemi(sg7884)", CENTER_MODE);
  HAL_Delay(200);
}

void Recording_Screen()
{
  lcd.Clear(LCD_COLOR_WHITE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Recording...", CENTER_MODE);
  //lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"To unlock,", CENTER_MODE);
  char buf[50];
  snprintf(buf, 50, "Distance: %3.2f m", totalDist);
  lcd.DisplayStringAt(0, LINE(9), (uint8_t *)buf, CENTER_MODE);
  char buf2[50];
  snprintf(buf2, 50, "Velocity: %3.2f m/s", currentVelocity * 100);
  lcd.DisplayStringAt(0, LINE(10), (uint8_t *)buf2, CENTER_MODE);
  //snprintf(buf, 50, "Distance: %3.2f m", totalDist);
  lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Press button to reset", CENTER_MODE);
  HAL_Delay(200);
}

void FINISH_Screen()
{
  lcd.Clear(LCD_COLOR_WHITE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"【Recording Finished】", CENTER_MODE);
  
  char buf[50];
  snprintf(buf, 50, "Distance: %3.2fm", totalDist);
  lcd.DisplayStringAt(0, LINE(9), (uint8_t *)buf, CENTER_MODE);

  char buf2[50];
  snprintf(buf2, 50, "Avg Veloci: %3.2fm/s", totalDist/RECORDING_TIME);
  lcd.DisplayStringAt(0, LINE(10), (uint8_t *)buf2, CENTER_MODE);

  lcd.DisplayStringAt(0, LINE(13), (uint8_t *)"Press button", CENTER_MODE);
   lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"to restart", CENTER_MODE);
  HAL_Delay(200);
}