#include <c8051f120.h>
#include <ADS1015.h>
#include <intrins.h>
#include <stdio.h>
/***周期单位：s*/
/*----------------------注：SYSCLK最好选3.0625MHz,不要选定时器1作为串口时钟源----------*/
/***************************************/
#define Reset_Sources_Init() do{WDTCN = 0xde;WDTCN = 0xad;}while(0)

#define Interrupts_Init  (IE = 0x90)

#define cli (IE &= ~0x10)

#define sti (IE |= 0x10)

#define BUFLEN 0x11 

#define ADDR 0x90

#define DAT 0x14                          
/*******子函数声明***************/
void UART_Init(void); 

void Port_IO_Init(void);

void Timer_Init(void);

void sleep(int sec);

void SendMessage(unsigned char message);

void I2C_start(void);

void I2C_stop(void);

unsigned char PCF8591_RD(void);

bit Acknowledge(void);

void Init_PCF8591(void);

void Bub_Sort(unsigned char *sort);

unsigned char filter(unsigned char *array);
/*****************************/

/***********全局变量*********/
bit flag = 0;
/****************************/

/**********寄存器定义********/
sbit dbg = P0^4;

sbit SDA = P2^0;

sbit SCL = P2^1;
/****************************/
int main(void)
{
   unsigned char dat,zeropoint,adval[DAT];
   Reset_Sources_Init();
   Port_IO_Init();
   Timer_Init();
   UART_Init();
   Init_PCF8591();
   Interrupts_Init;
   
   zeropoint = PCF8591_RD();
   while (1)
  {	
    dat = filter(adval);
    if (flag)
   {
	 cli; 
     TI0 = 1;
     printf("t6.txt=\"%d\"",(int)dat);
     printf("%c%c%c",0xff,0xff,0xff);
     TI0 = 0;
	 sti;
   }
  }
}

void Timer_Init(void)
{
    SFRPAGE   = TMR2_PAGE;
    TMR2CN    = 0x04;
    TMR2CF    = 0x08;
    RCAP2L    = 0xEC;
    RCAP2H    = 0xFF;
} 

void I2C_start(void)
{
  SDA = 1;
  sleep(7);
  SCL = 1;
  sleep(7);
  SDA = 0;
  sleep(7);
  SCL = 0;
}

void I2C_stop(void)
{
  SDA = 0;
  sleep(6);
  SCL = 1;
  sleep(6);
  SDA = 1;
}

void SendMessage(unsigned char message)
{
  unsigned char datas = message,cnt; 
 for (cnt = 0x0;cnt < 0x8;cnt++)
 {
    SCL = 0;
    datas <<= 1;
	SDA = CY;
	sleep(5);
	SCL = 1;
 }
  SCL = 0;
  sleep(5);
  SDA = 1;
  sleep(5);
}

void Init_PCF8591(void)
{
   I2C_start();
   SendMessage(ADDR);  //初始化AD
    Acknowledge();
   SendMessage(0x0);
    Acknowledge();
	sleep(10);
   I2C_stop();
}

unsigned char PCF8591_RD(void)
{ 
   unsigned char datas,count;
   bit carry;

   cli;
   I2C_start();
   SendMessage(ADDR+1);
   Acknowledge(); 
    
 for (count = 0x0;count < 0x8;count++)
 {
    SCL = 0;
    carry = SDA;
	datas = (datas << 1) | carry; 
	sleep(5);
	SCL = 1;
 }

  SCL = 1;
  SDA = 1;
  sleep(5);
  SCL = 0;

 I2C_stop();

 sti;
 return datas;
}

void UART_Init(void)
{
  SFRPAGE   = UART0_PAGE;
  SCON0     = 0x50;
  SSTA0     = 0x05;
}

void Port_IO_Init(void)
{
    SFRPAGE   = CONFIG_PAGE;
    P0MDOUT   = 0xf1;
    XBR0      = 0x04;
    XBR2      = 0x40;
}

void recieve(void) interrupt 4 using 2
{
  char dat;

   if (RI0)
 {
	dat = SBUF0;
  if (dat == 0x55)
     flag = 1;
    
  if (dat == 0xaa)
	flag = 0;

	RI0 = 0;
 }	
}

bit Acknowledge(void)
{
  bit ack;
  int timeout;
  SCL = 1;
  sleep(10);

 while (ack && timeout < 0xff); 
 {
  ack = SDA;
  timeout++;
 }
 
  SCL = 0;
  return ack;
} 

void sleep(int sec)
{
  int cnt;
  for (cnt = 0;cnt < sec;cnt++)
  	_nop_();
}

void Bub_Sort(unsigned char *sort)
{
   register char cnt1,cnt2;
   register unsigned char tmp;
  for (cnt1 = 0;cnt1 < DAT - 1;cnt1++)
 {
 	for (cnt2 = 0;cnt2 < DAT - 1 - cnt1;cnt2++)
 	{
 		if (sort[cnt2] > sort[cnt2+1])
 		{
 			tmp = sort[cnt2];
 			sort[cnt2] = sort[cnt2+1]; //可以考虑堆栈 
 			sort[cnt2+1] = tmp;
		}
	}
 }	
}

unsigned char filter(unsigned char *array)
{
  register unsigned char i;
  for (i = 0x0;i < DAT;i++)
 {
  	array[i] = PCF8591_RD();
	sleep(80);
 }

 Bub_Sort(array);

 return ((array[DAT>>1] + array[DAT>>1+1]) >>1);
}