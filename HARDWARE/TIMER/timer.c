/************************************************************
 *File		:	timer.c
 *Author	:  @YangTianhao ,490999282@qq.com��@TangJiaxin ,tjx1024@126.com	
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Timer5: 1ms, for Control_Task()
								Timer2: 1ms, for IMU Calculation
								Timer12: 50HZ, for Friction Wheel control
 ************************************************************/

#include "main.h"

#define Init_time 3000 

u16 TIM6_time_count = 0;  //TIM5��ʱ
u8 flag_Ready = 0; //��ʼ����ɱ�־
float adi_checksum=0;
float adi_checksum_pre=0;
u8 adi_die_flag=0;
u8 adi_die_count=0;

/*-------------  ��ʱ��5��ʼ��  -------------*/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 1000; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=90-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn ; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��5
	
}

/*-------------  ��ʱ��5�жϷ���  -------------*/
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET) //����ж�
	{			
		if(!flag_Ready)
		{
			if (TIM6_time_count < Init_time)   //��2�룬ʹ��ʼ����ɣ���ʹ��̨�Խ����ٶȹ��У���ʱ����PREPARE_STATEģʽ��������ң�������ݣ�������Remote_State_FSM()
			{
				TIM6_time_count++;
			}
			else 
			{			 
			 TIM6_time_count = 0;
			 flag_Ready = 1;
			}
		}
		continue_value();
			Control_Task();             //������ѭ��
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //����жϱ�־λ
}

/*-------------  ��ʱ��2��ʼ��  -------------*/
void TIM2_Init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 2000; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=90-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void TIM2_IRQHandler(void)      //2ms����һ���ж�
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{

		if(adi_die_count<10)
		{
			adi_die_flag=0;
		}
		else
		{
			update_euler_mpu();
			adi_die_flag=1;
		}
		
		if (adi_checksum_pre==adi_checksum)
		{
			if(adi_die_count<15)
				adi_die_count++;
		}
		else adi_die_count=0;
		adi_checksum_pre=adi_checksum;	
		adi_checksum=adis16470_real_data.gyro_x+adis16470_real_data.gyro_y+adis16470_real_data.gyro_z;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
	
}

///************************************************************
//*��ʱ��ʼ�������ڿ��ƶ��
//*************************************************************/
void TIM9_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitSruct;
	TIM_OCInitTypeDef TIM_OCInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  ///TIM3ʱ��ʹ��	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTHʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);
	
	TIM_TimeBaseInitSruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitSruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitSruct.TIM_Period=10000-1;
	TIM_TimeBaseInitSruct.TIM_Prescaler=180-1;
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitSruct);
	
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse =0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM9, ENABLE);
	TIM_SetCompare1(TIM9, COVER_CLOSE);
}

///************************************************************
//*???8???,???????
//*************************************************************/

void TIM5_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;


		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);      
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;          
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     
		GPIO_Init(GPIOA,&GPIO_InitStructure); 
		TIM_DeInit(TIM5);
		TIM_TimeBaseStructure.TIM_Period=1000-1;
		TIM_TimeBaseStructure.TIM_Prescaler=360-1;  
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);    
		TIM_OCStructInit(&TIM_OCInitStructure);     
		TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM5,ENABLE);
		TIM_Cmd(TIM5, ENABLE); 
		//TIM_SetCompare4(TIM5,500);

}



///************************************************************
//*��ʱ��4��ʼ��,����Ħ���ֲ���
//*************************************************************/


__INLINE void NVIC_IRQEnable(uint8_t irq, uint8_t pri, uint8_t subpri) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = (irq);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (pri);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = (subpri);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void TIM4_Init(void) 
	{
    
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
		
    TIM_TimeBaseInitTypeDef             TIM_TimeBaseStructure;
    TIM_ICInitTypeDef                   TIM_ICInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
    TIM_TimeBaseStructure.TIM_Period=50000-1;   // 20Hz
    TIM_TimeBaseStructure.TIM_Prescaler=90-1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV4;
    TIM_ICInitStructure.TIM_ICFilter = 0x10;        // Attention
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_DMACmd(TIM4, TIM_DMA_CC2|TIM_DMA_CC1, ENABLE);
    TIM_SetCounter(TIM4, 0);

    NVIC_IRQEnable(TIM4_IRQn, 0, 2);
    
    //TODO: TIM4_CH2/4 IRQ
    /* -------------- Configure DMA -----------------------------------------*/
    DMA_InitTypeDef dma;
		
    DMA_DeInit(DMA1_Stream0);
    dma.DMA_Channel = DMA_Channel_2;    // TIM4_CH1
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM4->CCR1);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH1.counters;   
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(Friction_CH1.counters)/sizeof(Friction_CH1.counters[0]);
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0, &dma);
    DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream0,ENABLE);
    
    DMA_DeInit(DMA1_Stream3);                     
    dma.DMA_Channel = DMA_Channel_2;    // TIM4_CH2
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM4->CCR2);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH2.counters;   
    dma.DMA_BufferSize = sizeof(Friction_CH2.counters)/sizeof(Friction_CH2.counters[0]);
    DMA_Init(DMA1_Stream3, &dma);
    DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream3,ENABLE);
    
    NVIC_IRQEnable(DMA1_Stream0_IRQn, 0, 1);
    NVIC_IRQEnable(DMA1_Stream3_IRQn, 0, 1);
    
    TIM_Cmd(TIM4, ENABLE);
}
	
void TIM4_IRQHandler(void)      //50ms����һ���ж�
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET) {
        Friction_CH1.stopping ++;
        if(Friction_CH1.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH1.speed, 
                sizeof(Friction_CH1.speed)/sizeof(Friction_CH1.speed[0]), 0, 15);
        #else
            Friction_CH1.speed[0] = 0;
        #endif
            Friction_CH1.stopping = 5;
        }
        
        
        Friction_CH2.stopping ++;
        if(Friction_CH2.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH2.speed, 
                sizeof(Friction_CH2.speed)/sizeof(Friction_CH2.speed[0]), 0, 15);
        #else
            Friction_CH2.speed[0] = 0;
        #endif
            Friction_CH2.stopping = 5;
        }
        
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void DMA1_Stream0_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0)) {
        
        dmaFrictionUpdata(&Friction_CH1);
        
        DMA_ClearFlag(DMA1_Stream0, DMA_IT_TCIF0);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {
        
        dmaFrictionUpdata(&Friction_CH2);
        
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
    }
}

