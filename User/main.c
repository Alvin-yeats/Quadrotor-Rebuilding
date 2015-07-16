int main()
{

dggj:
	RCC_Configuration();		//系统时钟初始化

	TIM1_PWM_Configuration();	//PWM初始化
	TIM2_AD_Configuration();	//AD初始化
	IM3_PWM_Configuration();	//PWM初始化
	MOTOR_PWM();				//占空比控制函数
	GPIO_Configuration();   	//GPIO配置

	EXTI3_Configuration();
	NVIC_Configuration();   	//中断配置
	DMA_Configuration();    	//DMA配置
	ADC1_Configuration();   	//ADC1配置

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	MOT_M=0;
	MOT_S=0;

	MS5607_Reset();
	Delay(50000);
	MS5607_ReadProm();

	ITG3205_sensor_init();
	DMA_Configuration();    //
	ADC1_Configuration();   //
	MC3220_sensor_init();
	ITG3205_sensor_read(ITG3205_GRY_txyz);

	Get_MS5607_Zero();

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	MOTOR_PWM();

	jiaoj=0;
	off=0;
	while (1)
	{
		mai_timenew=time_maileep;
		mai_timetemp=mai_timenew-mai_timelast;			
		if(mai_timetemp>=10)	
		{
			mai_timelast = mai_timenew; 
			if(ms4_cnt>200)
				ms4_cnt=0;
			if(jiaoj==30)
				goto dggj;

			xinhao();//3ms

			if(jiaoyan>0)
				GyroCorrectMag();

			Count_ADCResultProcess();
			MS5607_ReadPressure();
			Gain_LockModelPro();
			Contorller_Process();
		}

		if(ms4_cnt>39)
		{
			ms4_cnt=0;
			Throttle_hold();
		}

	}

	return 0;
}

void RCC_Configuration(void)
{
    RCC_DeInit();
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource()!=0x08);
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB ,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//72
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//36
}

void TIM1_PWM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	/* TIM1 configuration ------------------------------------------------------*/ 
  /* Time Base configuration */
  TIM_DeInit(TIM1);         //ÖØÐÂ½«TimerÉèÖÃÎªÈ±Ê¡Öµ
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 2048;          
  TIM_TimeBaseStructure.TIM_Prescaler = 6;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  /* TIM1 channel1 configuration in PWM mode */
    TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  /* TIM1 channel2 configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse =0; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  /* TIM1 channel3 configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//   TIM1 channel4 configuration in PWM mode 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  /* TIM1 main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

