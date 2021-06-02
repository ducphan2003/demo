#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "adc.h"
#include "stm32_dsp.h"
#include <math.h>
#include "pwm.h"
#include "dma.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define NPT					64	//FFT Sampling number

#define TIMING_ONE  68
#define TIMING_ZERO 32

uint32_t ADC_DataNum=0;			//ADC Sampling number

volatile uint8_t ADC_TimeOutFlag=1;			//ADC Time sampling time¡®s ok 	

extern __IO uint16_t ADCConvertedValue;		//ADC Sampling value

long lBUFMAG[NPT+NPT/2];					//Storing the data after modeling
long lBUFOUT[NPT];//FFT Output sequence
long lBUFIN[NPT];//FFT Output sequence

u8 count=1;
u8 count_old=5;
uint16_t colour_time=0;
u8 count_time=0;
uint16_t LED_BYTE_Buffer[14400];
uint8_t write[3]={0x0f,0x0f,0x0f};
uint8_t Green[3]={0x00,0x0f,0x00};
uint8_t red[3]={0x0f,0x00,0x00};
uint8_t blue[3]={0x00,0x00,0x0f};
uint8_t sendbuff[514][3]={0};//100 LED 
uint16_t DisplayDataBuf[32]={0};		//Display buffer

uint8_t fftHightRedBuf[NPT/2]={0};			// Red frequency column height array

void powerMag(long nfill)
{	 int32_t lX,lY; 
		uint32_t i; 
		for (i=0; i < nfill; i++) 
		{ 
			lX= (lBUFOUT[i]<<16)>>16; /* sine_cosine --> cos */ 
			lY= (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */     
			{ 
					float X=  64*((float)lX)/32768; 
					float Y = 64*((float)lY)/32768; 
					float Mag = sqrt(X*X+ Y*Y)/nfill;  //The first Sum of squares and root
				lBUFMAG[i] = (long)(Mag*65536); 
    }     
  } 


}
void WS2812_send(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr;
	uint16_t buffersize;
	buffersize = (len*24)+43;        // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;                                // reset buffer memory index

	while (len)
	{        
		for(i=0; i<8; i++) // GREEN data
		{
			LED_BYTE_Buffer[memaddr] = ((color[len][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
			LED_BYTE_Buffer[memaddr] = ((color[len][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
			LED_BYTE_Buffer[memaddr] = ((color[len][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		len--;
	}
	LED_BYTE_Buffer[memaddr] = ((color[0][2]<<8) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	memaddr++;        
	while(memaddr < buffersize)
	{
					LED_BYTE_Buffer[memaddr] = 0;
					memaddr++;
	}
	DMA_SetCurrDataCounter(DMA1_Channel3, buffersize);         // load number of bytes to be transferred
	DMA_Cmd(DMA1_Channel3, ENABLE);                         // enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE);                                                 // enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_FLAG_TC3)) ;         // wait until transfer complete
	TIM_Cmd(TIM3, DISABLE);         // disable Timer 3
	DMA_Cmd(DMA1_Channel3, DISABLE);                         // disable DMA channel 6
	DMA_ClearFlag(DMA1_FLAG_TC3);                                 // clear DMA1 Channel 6 transfer complete flag
}
int main(void)
{
	uint32_t i=0;
	uint16_t k=0;
	uint16_t m=0;
	uint16_t value=0;
	long adcx=0;
	delay_init();	    	 
	NVIC_Configuration(); 
	uart_init(9600);	
	TIM2_Configuration();
	TIM2_NVIC_Configuration();
	FFT_RCC_Configuration();
	FFT_GPIO_Configuration();
	FFT_ADC_Init();
	PWM_Init(89,0);
	 MYDMA_Config(DMA1_Channel3,(u32)&TIM3->CCR3,(u32)LED_BYTE_Buffer,14400);
	TIM_Cmd(TIM2, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	 
	while(1)
	{
		if(colour_time>=100)
		{
			colour_time=0;
			if(count_time<3)
			{
				if(Green[0]<0x0f)Green[0]++;
				if(Green[0]==0x0f) count_time=1;
				if(count_time>=1)
				{
					if(Green[1]<0x0f)Green[1]++;
					if(Green[1]==0x0f) count_time=2;
					if(count_time>=2)
					{
						if(Green[2]<0x0f)Green[2]++;
						if(Green[2]==0x0f) count_time=3;
					}
				}
			}
			if(count_time>=3)
			{
				if(Green[0]>0)Green[0]--;
				if(Green[0]==0&&Green[1]>0) Green[1]--;
				if(Green[1]==0&&Green[2])Green[2]--;
				if(Green[2]<0x03) count_time=0;
			}
		}
		if(ADC_TimeOutFlag)
		{
				ADC_TimeOutFlag=0;
				if(ADC_DataNum<NPT)
				{
					adcx=Get_Adc(13);
					lBUFIN[ADC_DataNum]=adcx;
					ADC_DataNum++;
				}
				else
				{
					TIM_Cmd(TIM2, DISABLE);
					ADC_DataNum=0;
					cr4_fft_64_stm32(lBUFOUT,lBUFIN,NPT);//the DSP library for FFT
					powerMag(NPT);//Calculated frequency amplitude
					for(i=0;i<514;i++)
					{
						for(m=0;m<3;m++)
						{
								sendbuff[i][m]=0x00;
						}
					}
					for(i=0;i<NPT/2;i++)
					{
						if(i==0) {if(lBUFMAG[i]>2700)DisplayDataBuf[i]=lBUFMAG[i]-2700;else DisplayDataBuf[i]=10;}
						else DisplayDataBuf[i]=lBUFMAG[i];
						if(i%2==0)	value=i*16+2;
						else value=i*16+2-15;
						for(m=0;m<3;m++)
						{
								sendbuff[value][m]=Green[m];
						}
						if(DisplayDataBuf[i]>1200)k=16;
						else if(DisplayDataBuf[i]>1000)k=15;
						else if(DisplayDataBuf[i]>900)k=14;
						else if(DisplayDataBuf[i]>800)k=13;
						else if(DisplayDataBuf[i]>700)k=12;
						else if(DisplayDataBuf[i]>600)k=11;
						else if(DisplayDataBuf[i]>500)k=10;
						else if(DisplayDataBuf[i]>400)k=9;
						else if(DisplayDataBuf[i]>300)k=8;
						else if(DisplayDataBuf[i]>250)k=7;
						else if(DisplayDataBuf[i]>210)k=6;
						else if(DisplayDataBuf[i]>180)k=5;
						else if(DisplayDataBuf[i]>150)k=4;
						else if(DisplayDataBuf[i]>120)k=3;
						else if(DisplayDataBuf[i]>100)k=2;
						else if(DisplayDataBuf[i]>80)k=1;
						else if(DisplayDataBuf[i]>20)k=0;
						for(;k>0;k--)
						{
							for(m=0;m<3;m++)
							{
								if(i%2==0)	
									sendbuff[value+k][m]=Green[m];
								if(i%2==1)	
									sendbuff[value-k][m]=Green[m];
							}
						}
						
					}
					delay_ms(50);
					WS2812_send(sendbuff,514);
					TIM_Cmd(TIM2, ENABLE);
			}
		}
	}
 }
 
 
void TIM2_IRQHandler(void)
{
   	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET){
			TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);	
			colour_time++;
	   ADC_TimeOutFlag=1;
	}
}

