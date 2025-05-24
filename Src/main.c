//PC0 -PC7 - D0-D7 IN LCD - Output
//PB5 -RS - Output
//PB6 -RW - Output
//PB7 -EN - Output

#include <stdint.h>
#include "stm32f411xx.h"

#define RS 0x20 //0010 0000
#define RW 0x40 //0100 0000
#define EN 0x80 //1000 0000

unsigned char Message1[]=("HELLO WORLD...");
unsigned char Message2[]=("HAPPY CODING...");

void GPIO_Inits(void);
void LCD_Command(unsigned char command);
void LCD_Data(unsigned char data);
void LCD_Init(void);
void delayMs(int delay);


int main(void)
{

	LCD_Init();
	/* Loop forever */

	while(1)
	{
		unsigned char i=0,j=0;
		LCD_Command(0x80); //First line Initialization (turn display)
		delayMs(15);
		while (Message1[i]!='\0')
		{
			LCD_Data(Message1[i]);
			i++;
		}
		delayMs(50);

		LCD_Command(0xc0); //Second line Initialization (turn display)
		delayMs(15);
		while (Message2[j]!='\0')
		{
			LCD_Data(Message2[j]);
			j++;
		}
		delayMs(50);

		LCD_Command(0x01); //clear screen move cursor home
		delayMs(15);
	}
}

void GPIO_Inits(void)
{
	RCC->AHB1ENR |= 0X06; //0110 - GPIOCEN(2) , GPIOBEN(1)

	//PB5,PB6,PB7 - Output(01)
	GPIOB->MODER |=0X5400; //0101 0100 0000 0000 - (7654 3210)

	//SET EN(PB7) AND RW(PB6) LOW -- (pin 31-16)->RESET, (pin 15-0) -> SET
	GPIOB->BSRR =0x00C00000; // 0000 0000 1100 0000(RESET)

	//PC0 -PC7 - Output(01)
	GPIOC->MODER |=0X00005555; //0101 0101 0101 0101
}

void LCD_Command(unsigned char command)
{
	// RS=0 -> Command,  RW=0 -> Write
	GPIOB->BSRR=(RS|RW)<<16;

	//PC0 -PC7 => Output
	GPIOC->ODR=command;

	GPIOB->BSRR=EN ; //ENABLE EN
	delayMs(0);
	GPIOB->BSRR=EN<<16 ; //clear EN
	//delayMs(30);
}

void LCD_Data(unsigned char data)
{
	//RS=1 -> Data
	GPIOB->BSRR=RS;
	//RW=0 -> Write
	GPIOB->BSRR=RW<<16;

	//PC0 -PC7 => Output
	GPIOC->ODR=data;

	GPIOB->BSRR=EN ; //ENABLE EN
	delayMs(30);
	GPIOB->BSRR=EN<<16 ; //clear EN
	delayMs(30);
}

void LCD_Init(void)
{
	GPIO_Inits();
	//any one Command works well for wake up command - (0x33) or (0x32)
	LCD_Command(0x33); //Wake up
		delayMs(15);
	LCD_Command(0x32); //Wake up
		delayMs(15);
	LCD_Command(0x38); //select one line
		delayMs(15);
	LCD_Command(0x0c); //Display ON, set b it data mode
		delayMs(15);
	LCD_Command(0x06); //move the cursor left to right
		delayMs(15);
	LCD_Command(0x01); //clear screen move cursor home
		delayMs(15);
	delayMs(5000);
}

void delayMs(int delay)
{
	int i;
	for (;delay>0;delay--)
	{
		for(i=0;i<3195;i++)
		{

		}
	}
}
