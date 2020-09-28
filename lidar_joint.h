 

#include <DynamixelWorkbench.h>


/*******************************************************************************
* encoder
*******************************************************************************/

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

/*******************************************************************************
* servo
*******************************************************************************/
#define DEVICE_NAME ""
#define BAUDRATE  1000000


#define LEFT            0
#define RIGHT           1
#define FORWARDS true
#define BACKWARDS false

#define PWM_MIN 110
#define PWMRANGE 255

/*******************************************************************************
* Joint servo
*******************************************************************************/

DynamixelWorkbench dxl_wb;

bool is_moving        = false;
std_msgs::Int32 joint_trajectory_point;
uint8_t DXL_ID = 1;

/*******************************************************************************
* encoder
*******************************************************************************/

// add in the next 3 lines to fix min max bug
#undef min
inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }
inline double min(double a, double b) { return ((a)<(b) ? (a) : (b)); }

#undef max
inline int max(int a, int b) { return ((a)>(b) ? (a) : (b)); }
inline double max(double a, double b) { return ((a)>(b) ? (a) : (b)); }

uint16_t lPwm;
uint16_t rPwm;
float l;
float r;

const uint8_t L_PWM = 9;
const uint8_t L_BACK = 4;
const uint8_t L_FORW = 5;
const uint8_t R_BACK = 6;
const uint8_t R_FORW = 7;
const uint8_t R_PWM = 10;

volatile long encoderLeft = 0L;
volatile long encoderRight = 0L;
const byte LencoderPinA = 2;
const byte LencoderPinB = 14;
const byte RencoderPinA = 3;
const byte RencoderPinB = 15;
volatile bool Lfired;
volatile bool Lup;
volatile bool Rfired;
volatile bool Rup;
