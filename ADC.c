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

#define DAT 0x14 

#define bitshift 4                         
/*******子函数声明***************/
void UART_Init(void); 

void Port_IO_Init(void);

void Timer_Init(void);

void sleep(int sec);

void SendMessage(unsigned char message);

void I2C_start(void);

void I2C_stop(void);

bit Acknowledge(void);

void Bub_Sort(unsigned int *sort);

unsigned int filter(unsigned int *array);

unsigned int readADC_SingleEnded(unsigned char channel,unsigned char m_i2cAddress);

void writeRegister(unsigned char i2cAddress,unsigned char reg,unsigned int value);

unsigned int readRegister(unsigned char i2cAddress,unsigned char regs); 
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
   unsigned int dat,adval[DAT],zeropoint;
   Reset_Sources_Init();
   Port_IO_Init();
   Timer_Init();
   UART_Init();
   Interrupts_Init;
   
  zeropoint = readADC_SingleEnded(0x0,ADS1015_ADDRESS);
   while (1)
  {	
     
    dat = filter(adval);
    if (flag)
   {
	 cli; 
     TI0 = 1;
     printf("t6.txt=\"%d\"",zeropoint);
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
    SCL = 1;	
    datas <<= 1;
	SDA = CY;
	sleep(5);
	SCL = 0;
 }
  SCL = 0;
  sleep(5);
  SDA = 1;
}

void writeRegister(unsigned char i2cAddress,unsigned char reg,unsigned int value)
{
  unsigned char hibyte,lwbyte;
  I2C_start();
  SendMessage(i2cAddress);
  Acknowledge();
  SendMessage(reg);
  Acknowledge();
  SendMessage((unsigned char)(value >> 8));
  Acknowledge();
  SendMessage((unsigned char)(value & 0xff));
  Acknowledge();
  I2C_stop();
}

unsigned int readRegister(unsigned char i2cAddress,unsigned char regs) 
{
  unsigned int ret;
  unsigned char lbyte;
  char cnt3;
  I2C_start();
  SendMessage(i2cAddress);
  Acknowledge();

  SendMessage(regs);
  Acknowledge();

 for(cnt3 = 0x0;cnt3 < 0x8;cnt3++)
{
   SCL = 0;
   ret = (ret << 1) | SDA;
   sleep(5);
   SCL = 1;
}
   Acknowledge();
   
 for(cnt3 = 0x0;cnt3 < 0x8;cnt3++)
{
   SCL = 0;
   lbyte = (lbyte << 1) | SDA;
   sleep(5);
   SCL = 1;
}

  Acknowledge();
  ret = (ret << 8) | lbyte;

  I2C_stop();

  return ret;  
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

void Bub_Sort(unsigned int *sort)
{
   register char cnt1,cnt2;
   register unsigned int tmp;
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

unsigned int filter(unsigned int *array)
{
  register unsigned char i;
  for (i = 0x0;i < DAT;i++)
 {
  	array[i] = readADC_SingleEnded(0x0,ADS1015_ADDRESS);
	sleep(80);
 }

 Bub_Sort(array);

 return ((array[DAT>>1] + array[DAT>>1+1]) >>1);
}

unsigned int readADC_SingleEnded(unsigned char channel,unsigned char m_i2cAddress) 
{ 
  // Start with default values
 unsigned int config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                       ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                       ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                       ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                       ADS1015_REG_CONFIG_DR_128SPS    | // 128 samples per second (default)
                       ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
  int retn;

  if (channel > 3)
    	return 0;
  // Set PGA/voltage range
  config |= GAIN_ONE;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress,ADS1015_REG_POINTER_CONFIG,config);

  // Wait for the conversion to complete
  sleep(ADS1015_CONVERSIONDELAY);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  retn = readRegister(m_i2cAddress+1,ADS1015_REG_POINTER_CONVERT);
  return (retn >> bitshift);  
}