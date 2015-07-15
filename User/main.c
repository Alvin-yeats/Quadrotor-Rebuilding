int main()
{
	RCC_Configuration();		//系统时钟初始化

   	TIM1_PWM_Configuration();	//PWM初始化
    TIM2_AD_Configuration();	//AD初始化
    TIM3_PWM_Configuration();	//PWM初始化
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
    HMC5883L_sensor_init();
		
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