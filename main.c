/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/*
 PSEUDO

 1. Clock in data for entire row (use R1,G1,B1,R2,G2,B2 for data and CLK for clock)

2. OE high

3. Select line address (A,B,C,D)

4. LAT high

5. LAT low

6. OE low

*/

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>



#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "misc.h"
#include "delay.h" // http://patrickleyman.be/blog/stm32f407-delay-with-systick/


#define COLDEPTH 		3
#define BRIGHTNESS		100
#define ROWS 			32
#define COLUMNS 		64

#define R1PORT			GPIOB
#define R1PIN			GPIO_Pin_9
#define G1PORT			GPIOD
#define G1PIN			GPIO_Pin_6
#define B1PORT			GPIOD
#define B1PIN			GPIO_Pin_7
#define R2PORT			GPIOD
#define R2PIN			GPIO_Pin_4
#define G2PORT			GPIOD
#define G2PIN			GPIO_Pin_5
#define B2PORT			GPIOD
#define B2PIN			GPIO_Pin_2

#define ROW0PORT		GPIOD
#define ROW0PIN			GPIO_Pin_3
#define ROW1PORT		GPIOD
#define ROW1PIN			GPIO_Pin_0
#define ROW2PORT		GPIOD
#define ROW2PIN			GPIO_Pin_1
#define ROW3PORT		GPIOC
#define ROW3PIN			GPIO_Pin_11

#define CLKPORT			GPIOC
#define CLKPIN			GPIO_Pin_12
#define LEPORT			GPIOA
#define LEPIN			GPIO_Pin_15
#define OEPORT			GPIOC
#define OEPIN			GPIO_Pin_10



int main(void) {

	// variables
	int buffsize;
	uint8_t *matrixbuff; // my matrix buffer pointer :)
    int x, y; // coordinates for pixels
    int i; // for loop thru registers/row
    int counter = 0;
    int EN = 1;
    int rowcounter = 0;
    int colcounter = 0;

	SysTick_Init(); // used for delay functions...might change later STM tutorials.........gratatata
	InitMatrixPorts();
    // GPIO CONFIGURATION
    // performance optimization / code simplification https://electronics.stackexchange.com/questions/36838/set-stm32-gpio-clock-and-data-pins-as-fast-as-possible

	// using pins D2-D7 as data pins R1G1B1 R2G2B2 u send 32 bits through each because each corresponds to 2/12 LED driver chips of 16-bits
    // set these pins as OUTPUT
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // enable clock
//    GPIO_InitTypeDef GPIO_InitStructData;
//    GPIO_InitStructData.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15 | GPIO_Pin_14;
//    GPIO_InitStructData.GPIO_Mode = GPIO_Mode_OUT; // Mod out !
//    GPIO_InitStructData.GPIO_Speed = GPIO_Speed_50MHz; // 50 MHZ clock frequency .. I think this is fine?
//    GPIO_InitStructData.GPIO_OType = GPIO_OType_PP; // Push pull mod
//    GPIO_InitStructData.GPIO_PuPd = GPIO_PuPd_DOWN; // Pull up .. I think this is fine?
//    GPIO_Init(GPIOD, &GPIO_InitStructData); // start it upppp
//
//    // using pins E4-E10 as control pins A B C D LAT CLK OE
//    // set these pins as OUTPUT
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//    GPIO_InitTypeDef GPIO_InitStructControl;
//    GPIO_InitStructControl.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
//    GPIO_InitStructControl.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructControl.GPIO_Speed = GPIO_Low_Speed;
//    GPIO_InitStructControl.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructControl.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOE, &GPIO_InitStructControl);

    // set initial pin values D2-D7 -> 0 and E4-E9 -> 0 and E10 -> 1
    GPIO_ResetBits(R1PORT, R1PIN); // R1
    GPIO_ResetBits(G1PORT, G1PIN); // G1
    GPIO_ResetBits(B1PORT, B1PIN); // B1
    GPIO_ResetBits(R2PORT, R2PIN); // R2
    GPIO_ResetBits(G2PORT, G2PIN); // G2
    GPIO_ResetBits(B2PORT, B2PIN); // B2

    GPIO_ResetBits(ROW0PORT, ROW0PIN); // A
    GPIO_ResetBits(ROW1PORT, ROW1PIN); // B
    GPIO_ResetBits(ROW2PORT, ROW2PIN); // C
    GPIO_ResetBits(ROW3PORT, ROW3PIN); // D

    GPIO_ResetBits(LEPORT, LEPIN); //
    GPIO_ResetBits(OEPORT, OEPIN); //
    GPIO_ResetBits(CLKPORT, CLKPIN); // display is disabled

    // delay_nms(20000);

    // create buffer
//    buffsize = 32*8*3; // not sure apparently x3 = 3 bytes holds 4 planes packed...?
//    matrixbuff = (uint8_t*)calloc(buffsize, sizeof(uint8_t));//allocate for single buffer calloc initializes to 0!!! ha
//    if(matrixbuff == NULL) // error handling
//    {
//    	printf("Error! memory not allocated.");
//        exit(0);
//    }



    while(1)
    {
        GPIO_ResetBits(R1PORT, R1PIN); // R1
        GPIO_ResetBits(G1PORT, G1PIN); // G1
        GPIO_ResetBits(B1PORT, B1PIN); // B1
        GPIO_ResetBits(R2PORT, R2PIN); // R2
        GPIO_ResetBits(G2PORT, G2PIN); // G2
        GPIO_ResetBits(B2PORT, B2PIN); // B2

//        for (rowcounter=0 ; rowcounter<(ROWS/2) ; rowcounter++)
//        {
//        	GPIO_SetBits(OEPORT,OEPIN);
//        	for (colcounter=0 ; colcounter<COLUMNS ; colcounter++)
//        	{
//        		GPIO_ResetBits(CLKPORT,CLKPIN);
//
//        		GPIO_SetBits(R1PORT,R1PIN);
//        		GPIO_SetBits(G1PORT,G1PIN);
//        		GPIO_SetBits(B1PORT,B1PIN);
//
//        		GPIO_SetBits(CLKPORT,CLKPIN);
//        		//if (colcounter>brightvalue) GPIO_ResetBits(OEPORT,OEPIN);
//        	}
//
//        	GPIO_SetBits(OEPORT,OEPIN);
//        	delay_nms(1);
//        	GPIO_SetBits(LEPORT,LEPIN);
//        	if ((rowcounter & 0x01)==0x01) GPIO_SetBits(ROW0PORT,ROW0PIN); else GPIO_ResetBits(ROW0PORT,ROW0PIN);
//        	if ((rowcounter & 0x02)==0x02) GPIO_SetBits(ROW1PORT,ROW1PIN); else GPIO_ResetBits(ROW1PORT,ROW1PIN);
//        	if ((rowcounter & 0x04)==0x04) GPIO_SetBits(ROW2PORT,ROW2PIN); else GPIO_ResetBits(ROW2PORT,ROW2PIN);
//        	if ((rowcounter & 0x08)==0x08) GPIO_SetBits(ROW3PORT,ROW3PIN); else GPIO_ResetBits(ROW3PORT,ROW3PIN);
//        	GPIO_ResetBits(LEPORT,LEPIN);
//        	//delay_nms(delaybrightness);
//        	delay_nms(1);
//        	GPIO_ResetBits(OEPORT,OEPIN);
//        	//delay_nms(59*(pwmindex-(8-COLDEPTH)+1)*(pwmindex-(8-COLDEPTH)+1)-59);
//        }

        for (rowcounter=0 ; rowcounter<(ROWS/2) ; rowcounter++)
        {

			GPIO_SetBits(OEPORT, OEPIN);

			// SHIFT IN
			for(i=0; i<64; i++)
			{
				GPIO_ResetBits(CLKPORT, CLKPIN);

				delay_nus(100);

				GPIO_SetBits(R1PORT, R1PIN);
				GPIO_SetBits(R2PORT, R2PIN);

				delay_nus(100);

				GPIO_SetBits(CLKPORT, CLKPIN);

			}

			// LATCH / ENABLE
			GPIO_SetBits(OEPORT, OEPIN);

			//delay_nus(100);

			GPIO_SetBits(LEPORT,LEPIN);
			if ((rowcounter & 0x01)==0x01) GPIO_SetBits(ROW0PORT,ROW0PIN); else GPIO_ResetBits(ROW0PORT,ROW0PIN);
			if ((rowcounter & 0x02)==0x02) GPIO_SetBits(ROW1PORT,ROW1PIN); else GPIO_ResetBits(ROW1PORT,ROW1PIN);
			if ((rowcounter & 0x04)==0x04) GPIO_SetBits(ROW2PORT,ROW2PIN); else GPIO_ResetBits(ROW2PORT,ROW2PIN);
			if ((rowcounter & 0x08)==0x08) GPIO_SetBits(ROW3PORT,ROW3PIN); else GPIO_ResetBits(ROW3PORT,ROW3PIN);
			GPIO_ResetBits(LEPORT, LEPIN);

			//delay_nus(100);

			//delay_nus(1000);

			GPIO_ResetBits(OEPORT, OEPIN);

			delay_nms(100);

			//delay_nus(1000);

        }
    }







    //GPIO_ResetBits(GPIOE, GPIO_Pin_10); // OE low -> turns it on






    // memset(matrixbuff, 0, buffsize);


//    if (x < 0 || x>31 || y < 0 || y>15) // error handling
//    {
//    	printf("Error! Pixel is off the grid");
//        exit(0);
//    }



    // ISR??????? FOR TIMER???? HOW???? -> REFRESH DISPLAAAYAAY



//    free(matrixbuff); // free memory






}


void InitMatrixPorts() {
	GPIO_InitTypeDef      GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG, ENABLE);

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = R1PIN;
  	GPIO_Init(R1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = G1PIN;
  	GPIO_Init(G1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = B1PIN;
  	GPIO_Init(B1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = R2PIN;
  	GPIO_Init(R2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = G2PIN;
  	GPIO_Init(G2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW3PIN;
  	GPIO_Init(ROW3PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW2PIN;
  	GPIO_Init(ROW2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW1PIN;
  	GPIO_Init(ROW1PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ROW0PIN;
  	GPIO_Init(ROW0PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = B2PIN;
  	GPIO_Init(B2PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = OEPIN;
  	GPIO_Init(OEPORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LEPIN;
  	GPIO_Init(LEPORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = CLKPIN;
  	GPIO_Init(CLKPORT, &GPIO_InitStructure);
}

void SysTick_Handler(void) {
	TimeTick_Decrement();
}
