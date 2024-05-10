#include "INCLUDE_H.h"
uint32_t flag1;//ֱ����������ת����־λ
uint32_t flag_range;//���̵����ݴ����־λ
uint32_t flag_resit_200,flag_resit_2k,flag_resit_20k;//����ı�־λ
uint32_t flag_timer;//��ʱ�������ı�־λ
uint32_t puiADC1Data[8],v_out,Rx;
uint32_t sum,sum_1;//sum�����ն�ȡ�����ŵ�ѹֵ//sum_1��ÿ�����е�����ȡ�ĵ�ѹ
uint32_t result;//ֱ����ѹ��ʵֵ
uint32_t result1;//������ѹƽ����
uint32_t v_value;//������ѹ����Чֵ
uint32_t a[100];//
uint32_t v_value_max;
uint32_t x;
uint32_t  result_new;
uint32_t  result_aver;

int k;


int t=0;
void TIMER_IRQHandler();//��ʱ���жϺ���
void ADC1IntHandler();//adc�жϺ���
void UART0IntHandler(void);//�����жϺ���
void delay();//��ʱ
void LCD();//��Ļ
void ADC_ACprogress();

//��ʱ����Ļ������
void delay(uint32_t n)
{
    for(uint32_t i = 0; i < n; i++)
        SysCtlDelay(SysCtlClockGet()/3000);
}

void LCD()
{ 
	UC1701Init(60000);
	UC1701Clear();
}

//���ڵ�����
uart_0()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
    UARTFIFODisable(UART0_BASE);
    
    UARTIntEnable(UART0_BASE, UART_INT_RX);
    UARTIntRegister(UART0_BASE, UART0IntHandler);
    IntEnable(INT_UART0);
    IntMasterEnable();

}
void UART0IntHandler(void)
{
    uint32_t ui32IntStatus;
    uint8_t ui8RxBuffer;

    ui32IntStatus = UARTIntStatus(UART0_BASE, true);

    UARTIntClear(UART0_BASE, ui32IntStatus);

    while (UARTCharsAvail(UART0_BASE))
    {

        ui8RxBuffer = (uint8_t) UARTCharGetNonBlocking(UART0_BASE);

    }
//		���߹���
//		A0����С��pa7
//		A1(��С)pf1 
//		A0(�Ŵ�)pa6 
//		A1(�Ŵ�)Pe1
		
    if (ui8RxBuffer =='1')//switch 1  ��Сʮ��
    {
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_PIN_7);//A0=1
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);//A1=0 
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0);//A0=0
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0);//A1=0
		
		flag_range=1;//ֱ�����̱�־λ
    }
		if (ui8RxBuffer == '2')//switch 2 ����
    {
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_PIN_7);//A0=1
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);//A1=0
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//A0=1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0);//A1=0
		
		flag_range=2;//ֱ�����̱�־λ
    }
		//adc�ɼ�������ѹ��λ
		if(ui8RxBuffer == '3')//������ѹ���䣬0.2��2V��λ
		{
			
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_PIN_7);//A0=1
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);//A1=0
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//A0=1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0);//A1=0
			flag_range=3;//������ѹ��־λ
			flag_timer=1;//����־λдΪ1��������ʱ��
		}
		if(ui8RxBuffer == '4')//������ѹ��С10����20v��λ
		{
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_PIN_7);//A0=1
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);//A1=0 
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0);//A0=0
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0);//A1=0
			flag_timer=1;//����־λдΪ1��������ʱ��
			flag_range=4;//������ѹ��־λ
		}	
		}

//���ڵĳ�ʼ��
//�������ŵ�����
void GPIO_SET_OUTPUT()
{
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
			GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_7);//A0����С��pa7
			GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);//A1(��С)pf1
			GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);//A0(�Ŵ�)pa6
			GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);//A0(�Ŵ�)pe1
	    
	  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,0);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);
	  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0);
      
}

//adc�ɼ���ѹ
 void ADC1IntHandler()
 { 
	 uint16_t i;
	 sum_1=0;
	 //ADCIntDisable(ADC1_BASE,0);
	 ADCIntClear(ADC1_BASE,0); 
	 ADCSequenceDataGet(ADC1_BASE,0,puiADC1Data);
	 for(i=0;i<8;i++)
	 {
		 sum_1=sum_1+(puiADC1Data[i]*3300/4096);
		 
	 }
	 sum=sum_1/8;
	
	 flag1=1;
	 //ADCIntEnable(ADC1_BASE,0);
 }
 void ADC1()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_INT_PIN_3);//PD3����Ϊadc��������
	
	ADCSequenceConfigure(ADC1_BASE,0,ADC_TRIGGER_PROCESSOR,0);
	ADCSequenceStepConfigure(ADC1_BASE,0,0,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,1,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,2,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,3,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,4,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,5,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,6,ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC1_BASE,0,7,ADC_CTL_CH4|ADC_CTL_IE|ADC_CTL_END);
	
	ADCIntEnable( ADC1_BASE,0);
	ADCIntRegister(ADC1_BASE,0,ADC1IntHandler);
	IntEnable(INT_ADC1SS0);
	
	IntMasterEnable();
	ADCSequenceEnable(ADC1_BASE,0);
	ADCIntClear(ADC1_BASE,0); 
}
//adc��ȡ��ѹ�����ݴ���
void V_Clc()
{
if(flag_range==1)//��ѹ��Сʮ���Ĵ���
	{
		
    result=((sum*3)-4500)*10;
    
		result_new=(result+227.3)/2.25;
	}
	if(flag_range==2)//��ѹ����Ĵ���
	{
    result=((sum*3)-4500);
		x=(result-0.895)/1.001;
	}
}
	
void ADC_progress()
	{
			ADCProcessorTrigger(ADC1_BASE,0);
		 
	if(flag1==1)
		{	if(flag_range==1)
			{
			Floatprint(2,6,result_new);
			flag1=0;
			result=0;
			result_new=0;
			}
			if(flag_range==2)
			{
				Floatprint(2,6,result);
			flag1=0;
			}
		}
		delay(200);
	}		
	
//adc�⽻����ѹ����ʱ����adc���ݴ���
//void Timer_Config(void)
//{
//  SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER0);
//	TimerConfigure( TIMER0_BASE,TIMER_CFG_A_PERIODIC_UP);

//	TimerDisable(TIMER0_BASE,TIMER_A);//���ö�ʱ��A
//	TimerLoadSet( TIMER0_BASE,  TIMER_A,SysCtlClockGet()/10-1);//װ��ʱ��һ��
//	TimerIntRegister( TIMER0_BASE,  TIMER_A,TIMER_IRQHandler);
//	TimerIntEnable( TIMER0_BASE,  TIMER_TIMA_TIMEOUT);
//	IntPrioritySet( INT_TIMER0A,  0);
//	
//	IntEnable( INT_TIMER0A);
//	IntMasterEnable();
//	TimerEnable( TIMER0_BASE, TIMER_A);
//}
//void TIMER_IRQHandler()
//{
//		if(TimerIntStatus(TIMER0_BASE,true));
//			{
//				a[t]=sum;//��ȡ��ʱ�̵�ѹֵ
//				t++;
//			}
//			if(t==100)//��ȡ100����ĵ�ѹ
//			{
//			  ADC_Clc();
//				t=0;
//			}
//			TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);		
//}
//  void  ADC_Clc()
//	{
//		int i;
//		int k;
//		k=5625;//kΪƫ��ֵ
//		result1=0;
//   for(i=0;i<100;i++)
//		{
//		  result1+=(a[i]-k)*(a[i]-k);//ƽ�������
//		}
//	flag1=2;
//	}	
//void ADC_ACprogress()
//	{
//			ADCProcessorTrigger(ADC1_BASE,0);
//			if(flag1==2)
//	{	
//				if(flag_range=3)//��ѹ����
//	    {
//    v_value=(sqrt(result1/100));
//		flag_range=0;
//	    }
//			
//    	if(flag_range=4)//��ѹ��Сʮ��
//	   {  
//    v_value=(sqrt(result1/100))*10;//����ѹ�Ŵ�ʮ��
//		flag_range=0;
//	   }
//		Floatprint(3,6,v_value);
//		 v_value_max=v_value*1.414;
//		Floatprint(2,6,v_value_max);
//		flag1=0;
//		sum_1=0;
//		v_value=0;
//	}
//	
//		delay(1000);	
//	}	
//void timer_Open_Set()
//{
//  		if(flag_timer==1)
//	{
//	  TimerEnable(TIMER0_BASE,TIMER_A);
//	}
//	    if(flag_timer==0)
//	{
//	  TimerDisable(TIMER0_BASE,TIMER_A);
//	}

//}
//	
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
    GPIO_SET_OUTPUT();//��������
    uart_0();//��������
	  LCD();
	  ADC1();
	 // Timer_Config();
    while (1)
    {		  
			V_Clc();
	    //timer_Open_Set();
			//ADC_ACprogress();
			ADC_progress();
		}
}


