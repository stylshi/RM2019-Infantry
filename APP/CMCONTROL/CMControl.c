/****************************************************************************************************
 *File		:	CMControl.c
 *Author	:  @YangTianhao ,490999282@qq.com��@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Control Chassis Motors.
								CMControlLoop() shows the way to control the motion of chassis in different states.
                Use PID to optimize Chassis motor control.
 *****************************************************************************************************/

#include "main.h"

int16_t CMFollowVal=0; 			    //���̸���ֵ
int16_t speedA_final,speedB_final,speedC_final,speedD_final;
//PID_Struct CMPid,CMFollowPid,CMFollowPid_speed;                 //�����˶�pid������pid
float step=100,keymove_x=0,keymove_y=0,m=0,n=0,q=0;           //step-�ٶȱ仯��  x-x�᷽��仯ֵ y-y�ٶȱ仯ֵ;




u8 quick_spin_flag=0;
int16_t CMFollowVal_QUICK=0;


PID_Type SPFOLLOW,SPCHIASSISA,SPCHIASSISB,SPCHIASSISC,SPCHIASSISD,SPFOLLOW_SPEED;


float max_output_speed=7500;

float singl_max=4000;
float speed_step=500;
float output_current_sum=0;

float spin_y=0;
float spin_x=0;

u8 climb_mode_flag=0;


float cm_normal_p=8.0;
float cm_normal_i=4.2;
float cm_normal_d=0;

float cm_climb_p=8.5;
float cm_climb_i=20;
float cm_climb_d=0.0;

float rotate_speed=250;
int rotate_speed_dir=1;
float rotate_change_para=1;
int rotate_counter=0;
int rotate_counter_100ms=0;
/*-------------  ���̿���ѭ��  -------------*/
void CMControlLoop(void)
{

    CM_Switch_Moni();
    if(remoteState == PREPARE_STATE)
    {
        CMControlOut(0,0,0,0);
    }
    else if(remoteState == NORMAL_REMOTE_STATE)
    {
			if(RC_Ex_Ctl.rc.s1 == 1)
			{
					CMFollowVal=rotate_speed*rotate_speed_dir;
				  float speedX = RC_Ex_Ctl.rc.ch0*3;
				  float speedY = RC_Ex_Ctl.rc.ch1*3;
					float theta=position_yaw_relative/8192.0f*2*PI;
					spin_x=speedX*cos(theta)-speedY*sin(theta);
					spin_y=speedX*sin(theta)+speedY*cos(theta);
					key_move(spin_x,spin_y,CMFollowVal);
			}
			else if(RC_Ex_Ctl.rc.s1 == 2)
			{
				CMFollowVal = followValCal(0);
				move(RC_Ex_Ctl.rc.ch0,RC_Ex_Ctl.rc.ch1+100,CMFollowVal/*0.5f*(RC_Ex_Ctl.rc.ch2)*/);
			}
			else
			{
				CMFollowVal = followValCal(0);
        move(RC_Ex_Ctl.rc.ch0,RC_Ex_Ctl.rc.ch1,CMFollowVal/*0.5f*(RC_Ex_Ctl.rc.ch2)*/);
			}
        
    }
    else if(remoteState == STANDBY_STATE )
    {
        CMStop();
    }
    else if(remoteState == ERROR_STATE )
    {
        CMStop();
    }
    else if(remoteState == KEY_REMOTE_STATE )
    {
				if(rotate_counter<10000)
				{
					rotate_counter++;
				}
				else
				{
					rotate_counter=0;
				}
				if(rotate_counter_100ms<1000)
				{
					if(rotate_counter%100==0)
					{
						rotate_counter_100ms++;
					}
				}
				else
				{
					rotate_counter_100ms=0;
				}
				
				
        if(clearing_flag==0)
        {
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
            {
                CM_Normal_PID();
                keyboardmove(RC_Ex_Ctl.key.v,1500,1500);//С�����ƶ���
            }
            else if(climb_mode_flag==1)
            {
                CM_Climb_PID();
                keyboardmove(RC_Ex_Ctl.key.v,2000,2500);//�������µ�λ
            }
            else
            {
                CM_Normal_PID();
                keyboardmove(RC_Ex_Ctl.key.v,8000,8000);//�����˶���
            }

            //С����ȫ��ת��
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_W )!=KEY_PRESSED_OFFSET_W
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_A )!=KEY_PRESSED_OFFSET_A
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_S )!=KEY_PRESSED_OFFSET_S
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_D )!=KEY_PRESSED_OFFSET_D)
            {
								rotate_change_para=sin(rotate_counter_100ms);
                CMFollowVal=(rotate_speed+300/**fabs(rotate_change_para)*/)*rotate_speed_dir;
                key_move(keymove_x,keymove_y,CMFollowVal);
            }
            //С������ת�ƶ���
            else if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
            {
                CMFollowVal=rotate_speed*rotate_speed_dir;
                float theta=position_yaw_relative/8192.0f*2*PI;
                spin_x=keymove_x*cos(theta)-keymove_y*sin(theta);
                spin_y=keymove_x*sin(theta)+keymove_y*cos(theta);
                key_move(spin_x,spin_y,CMFollowVal);
            }
            //�����˶���
            else
            {
//								rotate_change_para=sin(rotate_counter);
                CMFollowVal = followValCal(0);
                key_move(keymove_x,keymove_y,CMFollowVal);
            }
        }
        else
        {
            CMStop();
        }
    }
    else if(remoteState == VIEW_STATE )
    {
        if(auto_aim_flag==0xFF)
        {
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
            {
                CM_Normal_PID();
                keyboardmove(RC_Ex_Ctl.key.v,1500,1500);//С�����ƶ���
            }
            else if(climb_mode_flag==1)
            {
                CM_Climb_PID();
                keyboardmove(RC_Ex_Ctl.key.v,2000,2500);//�������µ�λ
            }
            else
            {
                CM_Normal_PID();
                keyboardmove(RC_Ex_Ctl.key.v,8000,8000);//�����˶���
            }

            //С����ȫ��ת��
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_W )!=KEY_PRESSED_OFFSET_W
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_A )!=KEY_PRESSED_OFFSET_A
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_S )!=KEY_PRESSED_OFFSET_S
                    &&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_D )!=KEY_PRESSED_OFFSET_D)
            {
                CMFollowVal=(rotate_speed+300)*rotate_speed_dir;
                key_move(keymove_x,keymove_y,CMFollowVal);
            }
            //С������ת�ƶ���
            else if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
            {
                CMFollowVal=rotate_speed*rotate_speed_dir;
                float theta=position_yaw_relative/8192.0f*2*PI;
                spin_x=keymove_x*cos(theta)-keymove_y*sin(theta);
                spin_y=keymove_x*sin(theta)+keymove_y*cos(theta);
                key_move(spin_x,spin_y,CMFollowVal);
            }
            //�����˶���
            else
            {
                CMFollowVal = followValCal(0);
                key_move(keymove_x,keymove_y,CMFollowVal);
            }
        }
        else
        {
            CMStop();
        }
    }


}




/*-------------  ����ֹͣ  -------------*/
void CMStop(void)
{
    CAN1_Send_Bottom(0,0,0,0);
}


/***************************************************************************************
 *Name     : move
 *Function ��������̵���ٶȸ���ֵ ��ң�������ơ�ͬʱ�и������ӵ��ٶ�����
 *Input    ��speedX, speedY, rad
 *Output   ����
 *Description :�ı���ȫ�ֱ���	speedA, speedB, speedC, speedD
****************************************************************************************/
void move(int16_t speedX, int16_t speedY, int16_t rad)
{
    float max_speed=0;
    speedX *= gears_speedXYZ;
    speedY *= gears_speedXYZ;
    rad *= gears_speedRAD;


//		if(fabs(speedY)>6000&&fabs(rad)<100)
//		{
//			rad*=10;
//		}

    int16_t speedA = ( speedX + speedY + rad);
    int16_t speedB = ( speedX - speedY + rad);
    int16_t speedC = (-speedX - speedY + rad);
    int16_t speedD = (-speedX + speedY + rad);

    if(fabs(speedA)>max_speed)
        max_speed=fabs(speedA);
    if(fabs(speedB)>max_speed)
        max_speed=fabs(speedB);
    if(fabs(speedC)>max_speed)
        max_speed=fabs(speedC);
    if(fabs(speedD)>max_speed)
        max_speed=fabs(speedD);

    if(max_speed>max_output_speed)
    {
        speedA*=max_output_speed/max_speed;
        speedB*=max_output_speed/max_speed;
        speedC*=max_output_speed/max_speed;
        speedD*=max_output_speed/max_speed;
    }

    CMControlOut(speedA,speedB,speedC,speedD);
}


void key_move(int16_t speedX, int16_t speedY, int16_t rad)
{
    float max_speed=0;
    rad *= gears_speedRAD;

    if(fabs(speedY)>5000&&fabs(rad)<50&&fabs(rad)>5)
    {
        rad*=10;
    }
    int16_t speedA = ( speedX + speedY + rad);
    int16_t speedB = ( speedX - speedY + rad);
    int16_t speedC = (-speedX - speedY+ rad);
    int16_t speedD = (-speedX + speedY + rad);

    if(fabs(speedA)>max_speed)
        max_speed=fabs(speedA);
    if(fabs(speedB)>max_speed)
        max_speed=fabs(speedB);
    if(fabs(speedC)>max_speed)
        max_speed=fabs(speedC);
    if(fabs(speedD)>max_speed)
        max_speed=fabs(speedD);

    if(max_speed>max_output_speed)
    {
        speedA*=max_output_speed/max_speed;
        speedB*=max_output_speed/max_speed;
        speedC*=max_output_speed/max_speed;
        speedD*=max_output_speed/max_speed;
    }

    CMControlOut(speedA,speedB,speedC,speedD);
}



/***************************************************************************************
 *Name     : CMControlOut
 *Function �����̵���ٶ����������PID�����������͸������������ͬʱ�и������ӵ��ٶ�����
 *Input    ��speedA,speedB,speedC,speedD
 *Output   ����
 *Description : ���̵���ٶȻ��ͷ�ֵ����
****************************************************************************************/
int16_t out_a=0;
float lec_numA=3000;
float lec_numB=3000;
float lec_numC=3000;
float lec_numD=3000;

int16_t pre_current_A=0;
int16_t pre_current_B=0;
int16_t pre_current_C=0;
int16_t pre_current_D=0;
int16_t current_step=600;//80;
u8 super_cap_flag=0;
void CMControlOut(int16_t spa , int16_t spb ,int16_t spc ,int16_t spd )
{

    float current_sum=0;
    int max_now_speed=0;

    float speedA = PID_ControllerDriver(&SPCHIASSISA,spa,current_cm_201);
    float speedB = PID_ControllerDriver(&SPCHIASSISB,spb,current_cm_202);
    float speedC = PID_ControllerDriver(&SPCHIASSISC,spc,current_cm_203);
    float speedD = PID_ControllerDriver(&SPCHIASSISD,spd,current_cm_204);

    if(cap_receive.state_module==Cap_Run&&cap_receive.state_cap==Cap_Boost&&cap_receive.CapVol>11800)
    //if(1)
    {
        CAN1_Send_Bottom(speedA,speedB,speedC,speedD);
        super_cap_flag=0;
    }
    else
    {
        super_cap_flag=1;
        if(abs(current_cm_201)>max_now_speed)
            max_now_speed=abs(current_cm_201);
        if(abs(current_cm_202)>max_now_speed)
            max_now_speed=abs(current_cm_202);
        if(abs(current_cm_203)>max_now_speed)
            max_now_speed=abs(current_cm_203);
        if(abs(current_cm_204)>max_now_speed)
            max_now_speed=abs(current_cm_204);


        if(abs(speedA-pre_current_A)>current_step)
        {
            if(speedA-pre_current_A>current_step)
                speedA=pre_current_A+current_step;
            else if(speedA-pre_current_A<-current_step)
                speedA=pre_current_A-current_step;
        }

        if(abs(speedB-pre_current_B)>current_step)
        {
            if(speedB-pre_current_B>current_step)
                speedB=pre_current_B+current_step;
            else if(speedB-pre_current_B<-current_step)
                speedB=pre_current_B-current_step;
        }
        if(abs(speedC-pre_current_C)>current_step)
        {
            if(speedC-pre_current_C>current_step)
                speedC=pre_current_C+current_step;
            else if(speedC-pre_current_C<-current_step)
                speedC=pre_current_C-current_step;
        }
        if(abs(speedD-pre_current_D)>current_step)
        {
            if(speedD-pre_current_D>current_step)
                speedD=pre_current_D+current_step;
            else if(speedD-pre_current_D<-current_step)
                speedD=pre_current_D-current_step;
        }
        current_sum=fabs(speedA)+fabs(speedB)+fabs(speedC)+fabs(speedD);



        if(current_sum > max_output_current)
        {
            speedA*=(max_output_current)/current_sum;
            speedB*=(max_output_current)/current_sum;
            speedC*=(max_output_current)/current_sum;
            speedD*=(max_output_current)/current_sum;
        }
        CAN1_Send_Bottom(speedA,speedB,speedC,speedD);
    }

    output_current_sum=abs(speedA)+abs(speedB)+abs(speedC)+abs(speedD);
    pre_current_A=speedA;
    pre_current_B=speedB;
    pre_current_C=speedC;
    pre_current_D=speedD;


}


/**********  ���̵����ֵ����  **********/
float CMSpeedLegalize(float MotorCurrent , float limit)
{
    return MotorCurrent<-limit?-limit:(MotorCurrent>limit?limit:MotorCurrent);
}


/***************************************************************************************
 *Name     : followValCal
 *Function �����̵���������
 *Input    ��Setposition,Yaw�������̸���ֵ
 *Output   �����̸�����
 *Description : ���̵�����������㺯��
****************************************************************************************/
int16_t followValCal(float Setposition)
{
    int16_t followVal = 0;
    float NowPosition = position_yaw_relative;
    followVal=PID_ControllerDriver(&SPFOLLOW,Setposition,NowPosition);
//	float followVal_speed=PID_ControllerDriver(&SPFOLLOW_SPEED,followVal/20.0,yaw_speed);
//	followVal = CMSpeedLegalize(followVal_speed,1200);
    //��������Сֵ���Ƕȹ�С������
    //if(abs(followVal) < followVal_limit) followVal = 0;

    return followVal;
}



/*-------------  ���̵���ٶ�PID�͸���PID��ʼ��  -------------*/



/*���̳�ʼ��*/
void CMControlInit(void)
{
    PID_ControllerInit(&SPFOLLOW,50,150,1100,0.01);
    SPFOLLOW.intergration_separation = 100;

    PID_ControllerInit(&SPCHIASSISA,20,20,16384,0.01);
    PID_ControllerInit(&SPCHIASSISB,20,20,16384,0.01);
    PID_ControllerInit(&SPCHIASSISC,20,20,16384,0.01);
    PID_ControllerInit(&SPCHIASSISD,20,20,16384,0.01);
    PID_ControllerInit(&SPFOLLOW_SPEED,20,20,1500,0.01);
    PID_SetGains(&SPFOLLOW, 0.29,0.3,0.004);// 0.28,0.3,0.8
    PID_SetGains(&SPFOLLOW_SPEED,3,0.5,0.1);
    CM_Normal_PID();
}

void CM_Normal_PID(void)
{
    PID_SetGains(&SPCHIASSISA,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISB,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISC,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISD,cm_normal_p,cm_normal_i,cm_normal_d);
	PID_SetGains(&SPFOLLOW, 0.29,0.3,0.004);// 0.28,0.3,0.8
}

void CM_Climb_PID(void)
{
    PID_SetGains(&SPCHIASSISA,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISB,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISC,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISD,cm_climb_p,cm_climb_i,cm_climb_d);
	PID_SetGains(&SPFOLLOW, 0.23,0.3,0.002);// 0.28,0.3,0.8
}


void keyboardmove(uint16_t keyboardvalue,uint16_t xlimit,uint16_t ylimit)
{

    // W and S
    switch(keyboardvalue & (KEY_PRESSED_OFFSET_W|KEY_PRESSED_OFFSET_S))
    {
    case ( KEY_PRESSED_OFFSET_W):
        m = 0;
        if(keymove_y>0)
            keymove_y += 0.8f*step;
        else
            keymove_y += step*2;
        break;
    case ( KEY_PRESSED_OFFSET_S):
        m = 0;
        if(keymove_y<0)
            keymove_y -= 0.8f*step;
        else
            keymove_y -= step*2;
        break;
    default:
        m++;
        if(m>1)
        {
            if(keymove_y>3*step) {
                keymove_y=keymove_y-3*step;
            }
            else if(keymove_y<-3*step) {
                keymove_y=keymove_y+3*step;
            }
            else {
                keymove_y = 0;
            }
        }
        break;
    }

    //  A and D  stand for X axis
    switch(keyboardvalue & (KEY_PRESSED_OFFSET_A | KEY_PRESSED_OFFSET_D))
    {
    case ( KEY_PRESSED_OFFSET_A):
        n = 0;
        if(keymove_x<0)
            keymove_x -= 3*step;
        else
            keymove_x -= 3*step;
        break;
    case ( KEY_PRESSED_OFFSET_D):
        n = 0;
        if(keymove_x>0)    //��������
            keymove_x += 3*step;           //�Ӳ���
        else
            keymove_x += 3*step;
        break;

    default:
        n++;
        if(n>1)
        {
            if(keymove_x>3*step) {
                keymove_x=keymove_x-3*step;
            }
            else if(keymove_x<-3*step) {
                keymove_x=keymove_x+3*step;
            }
            else {
                keymove_x = 0;
            }
        }
        break;
    }
    keymove_x = keymove_x>xlimit?xlimit:(keymove_x<-xlimit?-xlimit:keymove_x);  //����
    keymove_y = keymove_y>ylimit?ylimit:(keymove_y<-ylimit?-ylimit:keymove_y);

    if(keymove_x<5&&keymove_x>-5) keymove_x = 0;
    if(keymove_y<5&&keymove_y>-5) keymove_y = 0;	 //��������

}

u8 cm_switch_delay=0;


int rotate_shift_count=0;
void CM_Switch_Moni(void)
{
    if(cm_switch_delay>0)
        cm_switch_delay--;


    if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_X )==KEY_PRESSED_OFFSET_X&&cm_switch_delay<=0)
    {
        if(climb_mode_flag==1)
            climb_mode_flag=0;
        else
            climb_mode_flag=1;
        cm_switch_delay=50;
    }

    if(	rotate_shift_count>-1)
    {
        rotate_shift_count--;
    }

    if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
    {
        rotate_shift_count=50;
    }
    if(rotate_shift_count==0&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )!=KEY_PRESSED_OFFSET_SHIFT)
    {
        rotate_speed_dir=-rotate_speed_dir;
    }


}







