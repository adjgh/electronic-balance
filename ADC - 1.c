#include <c8051f120.h>
#include <stdio.h>
/*----------------------ע��SYSCLK���ѡ3.0625MHz,��Ҫѡ��ʱ��1��Ϊ����ʱ��Դ----------*/
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
/*******�Ӻ�������***************/
void Timer_Init(void);

void UART_Init(void); 

void ADC_Init(void);

void Port_IO_Init(void);

unsigned int binary_search(LPRAM ptr,unsigned int seek);
/*****************************/

/***********ȫ�ֱ���*********/

unsigned long Result;//�з������������ݳ���

bit flag = 0;
/****************************/
/**********�Ĵ�������********/
sfr16 ADC0 = 0xbe;
/****************************/
int main(void)
{
  PRAM xdata table[] = {{0,53},{10,63},{20,82},{30,95},{40,107},{50,130},{60,147},{70,172},{80,195},{90,222},{100,245},{110,267},{120,286},{130,306},{140,329},{150,350},{160,370},{170,390},{180,415},{190,437},{200,460}};
   char str[8];
   unsigned int result;
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
     result = (unsigned int)Result;
     sprintf(str,"%ld",Result);
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
   REF0CN    = 0x02; //�ο���ѹ����
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

unsigned int binary_search(LPRAM ptr,unsigned int seek)
{
   char low = 0x0,high = POINT - 1,mid;
   while (low <= high)
{
  mid = (low + high) / 2;
  		if (ptr[mid].volt > seek)
	 {
	      if (ptr[mid].volt - seek < 10)
		   return ptr[mid].mass;
		   	else
				high = mid - 1;
	 }
				else if (ptr[mid].volt < seek)
			 {
			    if (seek - ptr[mid].volt < 10)
					return ptr[mid].mass;
					else	
						low = mid + 1;
			 }
}
}
