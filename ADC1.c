#include <c8051f120.h>
#include <stdio.h>
/*----------------------注：SYSCLK最好选3.0625MHz,不要选定时器1作为串口时钟源----------*/
/***************************************/
#define Reset_Sources_Init() do{WDTCN = 0xde;WDTCN = 0xad;}while(0)

#define Interrupts_Init  (IE = 0x90)

#define ADC_INT (EIE2 = 0x2)

#define cli (IE &= ~0x10)

#define sti (IE |= 0x10)

#define POINT 21

typedef struct param
{
  unsigned int mass;
  unsigned int volt;
}PRAM,*LPRAM;
/*******子函数声明***************/
void Timer_Init(void);

void UART_Init(void); 

void ADC_Init(void);

void Port_IO_Init(void);

void Flash_Config(void);

float seek(LPRAM lptr,unsigned int lseek);

float slope(unsigned int volt0,unsigned int volt1);
/*****************************/

/***********全局变量*********/
unsigned long Result;//有符号数导致数据出错

bit flag = 0;

bit calibration = 1;
/****************************/
/**********寄存器定义********/
sfr16 ADC0 = 0xbe;
/****************************/
int main(void)
{
   PRAM zero;
   PRAM xdata table[21]={{0,1310},{10,1315},{20,1320},{30,1324},{40,1330},{50,1335},{60,1340},{70,1345},{80,1350},{90,1355},{100,1360},{110,1366},{120,1371},{130,1376},{140,1381},{150,1386},{160,1391},{170,1396},{180,1401},{190,1407},{200,1412}};
   char str[8],i;
   unsigned int delta,dat;
   float retn;
   Reset_Sources_Init();
   Timer_Init();
   ADC_Init();
   Port_IO_Init();
   UART_Init();
   ADC_INT;
   Interrupts_Init;

   while (1)
  {	
    if (flag)
  {	
     if (calibration)
	{
     zero.volt = (unsigned int)Result;
	 calibration = 0;
	 if (zero.volt > table[0].volt)
	{
	 	delta = zero.volt - table[0].volt;
	  		for (i = 0x0;i < 0x15;i++)	 //更新表
	 			table[i].volt += delta;
    }
			else if (zero.volt < table[0].volt)
		{
			 	delta = table[0].volt - zero.volt;
	 			 	for (i = 0x0;i < 0x15;i++)
	 			       	table[i].volt -= delta;
		}
	}
	 dat = (unsigned int)Result;
	 retn = seek(table,dat); 
     sprintf(str,"%0.0f",retn);
     cli; 
     TI0 = 1;
     printf("t6.txt=\"%s\"",str);
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
	SFRPAGE   = TMR3_PAGE;
    TMR3CN    = 0x04;
    TMR3CF    = 0x08;
	TMR3L     = 0xfe;
	TMR3H     = 0xff;
	RCAP3L    = 0xfe;
	RCAP3H    = 0xff;
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
	P1MDOUT   = 0x01;
    XBR0      = 0x04;
    XBR2      = 0x40;
}

void ADC_Init(void)
{
   SFRPAGE   = ADC0_PAGE;
   ADC0CN    = 0x84;
   ADC0CF    = 0x28;
   REF0CN    = 0x02; //参考电压设置
}

void recieve(void) interrupt 4 using 1
{
  char dat;
  SFRPAGE   = UART0_PAGE;
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

void ADC0_ISR (void) interrupt 15 using 2
{
   static unsigned int int_dec = 8192;    // Integrate/decimate counter
                                       // we post a new result when
                                       // int_dec = 0
   static unsigned long accumulator=0;         // Here's where we integrate the
   
   SFRPAGE   = ADC0_PAGE;                                    // ADC samples
   EIE2 &= ~0x2;
   AD0INT = 0;                         // Clear ADC conversion complete
                                       // indicator

   accumulator += ADC0;                // Read ADC value and add to running
                                       // total
   int_dec--;                          // Update decimation counter

   if (int_dec == 0)                   // If zero, then post result
   {
      int_dec = 8192;               // Reset counter
      Result = accumulator >> 13;
      accumulator = 0L;                // Reset accumulator
   }
   EIE2 |= 0x2;
}

float seek(LPRAM lptr,unsigned int lseek)
{
    register char cnt;
    float tmp,flseek,fvolt,fmass;
  for (cnt = 0x0;cnt < 0x15;cnt++)
 {
      fmass = (float)lptr[cnt].mass;
	  fvolt = (float)lptr[cnt].volt;
	  flseek = (float)lseek;

	  if (lptr[cnt].volt == lseek)
	  	return (float)lptr[cnt].mass;
	  	else if (lptr[cnt+1].volt == lseek)
	  		return (float)lptr[cnt+1].mass;
	   if (lptr[cnt].volt < lseek && lptr[cnt+1].volt > lseek)
	 {
		 tmp =  slope(lptr[cnt].volt,lptr[cnt+1].volt) * (flseek - fvolt) + fmass;
		 break;
	 }
 }
  return tmp;
}

float slope(unsigned int volt0,unsigned int volt1)
{
  float num[2],tslope;
  
  num[0] = (float)volt0;
  num[1] = (float)volt1; 
  tslope = 10.0 / (num[1] - num[0]);
  return tslope;
}
