#include "stm32f10x.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "remote.h"   

#define KEYMODE_1   1  // 定义模式1
#define KEYMODE_2   2  // 定义模式2
#define KEYMODE_3   3  // 定义模式3
uint8_t keyMode;      //指无符号8Bit整形数

void IR_IN()                             //红外遥控子程序
{
	  u8 key;	
    ZYSTM32_brake(10);	
	   key=Remote_Scan();	
			switch(key)
			{    
				case 98:ZYSTM32_run(80,100);break;	    
				case 2:ZYSTM32_brake(100);break;		 	  
				case 194:ZYSTM32_Right(80,100);break;	   
				case 34:ZYSTM32_Left(80,100);break;		  
				case 224:ZYSTM32_Spin_Left(80,100);break;		  
				case 168:ZYSTM32_back(80,100);break;		   
				case 144:ZYSTM32_Spin_Right(80,100);break;		    
			}
}

void KeyScanTask(void)//按键子程序
{
	static u8 keypre = 0;//按键被按下时置1
	
	if((keypre == 0)&& (KEY))
	{
		keypre = 1; //置1，避免持续按下按键时再次进入此函数。
		switch(keyMode)
		{
			case KEYMODE_1:keyMode = KEYMODE_2; break;
			case KEYMODE_2:keyMode = KEYMODE_3; break;
			case KEYMODE_3:keyMode = KEYMODE_1; break;
			default: break;
		}
	}
	if(!KEY)  //按键被放开
	{
		keypre = 0;//置0，允许再次切换LED模式
	}
}

//任务：循迹、避障、遥控模式处理
void LEDTask()
{
		switch(keyMode)
		{
			case KEYMODE_1:
				LED_D4_SET;
			  LED_D3_RESET;
			  SearchRun();
			  break;
			case KEYMODE_2:
				LED_D4_RESET;
			  LED_D3_SET;
        AVoidRun();
			  break;
			case KEYMODE_3:
				LED_D4_SET;
			  LED_D3_SET;
        IR_IN();			
			  break;
			 default:
         break;
		}
}
	

//	  int val;
//  val=KEY;  //读取按键
//  while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))//当按键没有被按下时，一直循环
//  {
//    val=KEY;//词句可省略，可让循环跑空
//  }
//  while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))//当按键被按下时
//  {
//    delay_ms(10);	  //延时10ms
//    val=KEY;//读取数字端口PC2口电平值赋给Val
//    if(val==1)  //第二次判断按键是否被按下
//    {
//      BEEP_SET;		//蜂鸣器响
//      while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))	//判断按时是否被松开
//        BEEP_RESET;	//蜂鸣器停止
//    }
//    else
//				BEEP_RESET;//蜂鸣器停止
//  }
//}


 int main(void)
 {	
	 delay_init();
	 KEY_Init();
	 IRSearchInit();
	 IRAvoidInit();
	 TIM4_PWM_Init(7199,0);  //初始化PWM
	 Remote_Init();			    //红外接收初始化	
	 ZYSTM32_brake(500);
//	 keysacn();		
	 keyMode = KEYMODE_1;
		while(1)
		{  
		 
   KeyScanTask();
   LEDTask();			

		}
 }

