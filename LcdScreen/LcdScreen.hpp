#include <iostream>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>

#define RESET_PIN 10
#define CS_PIN 15
#define DC_PIN 9
#define BLK_PIN 13
#define	SPI_CHAN 0
//-----------------LCD端口定义---------------- 

#define LCD_RES_Clr()  digitalWrite(RESET_PIN,LOW)//RES
#define LCD_RES_Set()  digitalWrite(RESET_PIN,HIGH)

#define LCD_DC_Clr()   digitalWrite(DC_PIN,LOW)//DC
#define LCD_DC_Set()   digitalWrite(DC_PIN,HIGH)
 		     
#define LCD_CS_Clr()   digitalWrite(CS_PIN,LOW)//CS
#define LCD_CS_Set()   digitalWrite(CS_PIN,HIGH)

#define LCD_BLK_Clr()  digitalWrite(BLK_PIN,LOW)//BLK
#define LCD_BLK_Set()  digitalWrite(BLK_PIN,HIGH)

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t 

#define USE_HORIZONTAL 3  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 126
#define LCD_H 159

#else
#define LCD_W 159
#define LCD_H 126
#endif


void lcd_init(void);//LCD初始化
void bsp_init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void spi_send_frame(u8 *data,int sizeOfData);
