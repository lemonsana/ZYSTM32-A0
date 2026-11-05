#ifndef __KEYSACN_H_
#define __KEYSACN_H_

#include "stm32f10x.h"

void KEY_Init(void);//按键初始化
void keysacn(void);//按键扫描函数

//蜂鸣器IO口定义
//BEEP_ENABLE：蜂鸣器总开关。
//      0 = 关闭：所有 BEEP_SET 都变成空操作，PC3 保持低电平（静音）；
//      1 = 恢复原行为：避障/找线时鸣叫。
#define BEEP_ENABLE      0

#define BEEP_PIN         GPIO_Pin_3
#define BEEP_GPIO        GPIOC

#if BEEP_ENABLE
#define BEEP_SET         GPIO_SetBits(BEEP_GPIO,BEEP_PIN)
#define BEEP_RESET       GPIO_ResetBits(BEEP_GPIO,BEEP_PIN)
#else
#define BEEP_SET         ((void)0)     //关闭：不拉高，不响
#define BEEP_RESET       ((void)0)     //关闭：也不写寄存器，PC3 保持复位默认低电平
#endif

// LED D3灯IO口定义
#define LED_D3_PIN       GPIO_Pin_6
#define LED_D3_PIN_GPIO  GPIOB
#define LED_D3_SET       GPIO_SetBits(LED_D3_PIN_GPIO,LED_D3_PIN)
#define LED_D3_RESET     GPIO_ResetBits(LED_D3_PIN_GPIO,LED_D3_PIN)

// LED D4灯IO口定义
#define LED_D4_PIN       GPIO_Pin_6
#define LED_D4_PIN_GPIO  GPIOA
#define LED_D4_SET       GPIO_SetBits(LED_D4_PIN_GPIO,LED_D4_PIN)
#define LED_D4_RESET     GPIO_ResetBits(LED_D4_PIN_GPIO,LED_D4_PIN)


//读取按键
#define KEY  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)//读取按键0

#endif
