#ifndef __MAIN_H_
#define __MIAN_H_


#include "sp_math.h"
#include "sp_pid.h"
#include "ControlTask.h"
#include "bsp.h"
#include "buzzer.h"
#include "spi.h"
#include "can.h"
#include "delay.h"
#include "led.h"
#include "rc.h"
#include "timer.h"
#include "MonitorControl.h"
#include "RemoteControl.h"
#include "sp_kalman.h"
#include "adi_gyro.h"




#include "GimbalControl.h"









#define INFANTRY_4
#define Gimbal_Move


#define YawMax 1200 			
#define YawMin -1200					
#define PitMax 26	
#define PitMin -18

#define MIDDLE_YAW 	2800
#define MIDDLE_PITCH  2000

#define MOUSE_YAW_SPEED 0.0025   
#define MOUSE_PIT_SPEED 0.002



#endif




