#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f4xx.h"
//#include "sys.h"    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
void CAN1_Send_Bottom(int16_t candata1,int16_t candata2,int16_t candata3,int16_t candata4);
void CAN2_Cmd_SHOOT(int16_t current_207);
void CAN1_Cmd_standard(void);
void CAN1_Cmd_All(int16_t current_205,int16_t current_206);
void CAN1_Cmd_Confirm(int16_t current_205,int16_t current_206,int16_t current_207);
void CAN1_Cmd_Cap(u8 mode,int16_t set_val);
void Cap_Update_Cmd(u8 module_state, u8 cap_state,u8 val_change_state,u16 set_value);
void continue_value(void);
extern int count_207_temp;
void CAN2_Cmd_Float_Clear(void);
void CAN2_Send_Clear(int16_t clear_angle);
#define Cap_Run 1
#define Cap_Stop 0
#define Cap_Buck 1
#define Cap_Boost 0
#define Cap_change 1
#define Cap_Keep 0

typedef struct _super_cap_send
{
	u8 mode;
	u16 set_cur;
	u8 reserve[5];
}super_cap_send;

typedef struct _super_cap_receive
{
	u8 state_module;
	u8 state_cap;
	u16 CapVol;
	int16_t Imon;
	u8 reserve[3];
}super_cap_receive;

volatile extern int16_t current_position_206;    	//pitch gimbal
volatile extern int16_t current_position_205;			//yaw gimbal
volatile extern int16_t current_position_207;
volatile extern int16_t current_speed_207;
volatile extern int16_t current_cm_201;
volatile extern int16_t current_cm_202;
volatile extern int16_t current_cm_203;
volatile extern int16_t current_cm_204;
volatile extern int16_t current_cm_205;
volatile extern int16_t current_cm_206;
volatile extern int16_t continuous_current_position_205;			//yaw gimbal
volatile extern int16_t continuous_current_position_206;    	//pitch gimbal
volatile extern float continuous_current_position_207;
extern super_cap_receive cap_receive;
volatile extern int16_t t_i_1;
volatile extern int16_t t_i_2;
volatile extern int16_t t_i_3;
volatile extern int16_t t_i_4;
volatile extern int16_t continuous_current_position_filtered_205;			//yaw gimbal
volatile extern int16_t continuous_current_position_filtered_206;    	//pitch gimbal
extern int16_t pitch_history[20];
extern int16_t yaw_speed;
extern int16_t aid_dynamic_mach_angle;
#endif

















