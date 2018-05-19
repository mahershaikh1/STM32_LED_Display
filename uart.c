/**
  ******************************************************************************
  * @file    main.c

  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  * WORKS
  ******************************************************************************
*/

#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/* This function initializes the USART1 peripheral
 *
 */
void init_USART2(uint32_t baudrate){

	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	//EXTI_InitTypeDef EXTI_InitStruct;


	// IDK
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // using USART 2 so APB1

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // actually using A2(TX) and A3(RX)

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2 (TX) and 3 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART2 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART2_IRQHandler() function
	 * if the USART2 receive interrupt occurs
	 */
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt


//	/* PB12 is connected to EXTI_Line12 */
//	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
//	/* Enable interrupt */
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	/* Interrupt mode */
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	/* Triggers on rising and falling edge */
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	/* Add to EXTI */
//	EXTI_Init(&EXTI_InitStruct);


//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;		 // we want to configure the USART1 interrupts
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
//	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);





	// LED

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; // Led 6 Blue selecte
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // Mod out !
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 50 MHZ clock frequenc
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // Push pull mod
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // Pull up
	GPIO_Init(GPIOD, &GPIO_InitStruct);


}


/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

int main(void) {

  init_USART2(9600); // initialize USART1 @ 9600 baud


  USART_puts(USART2, "A"); // just send a message to indicate that it works
  int ch;

  while (1)
  {
	  //USART2_IRQHandler();
	  //GPIO_SetBits(GPIOD, GPIO_Pin_14);
	  //GPIO_ResetBits(GPIOD, GPIO_Pin_14);


	    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
	    {
	    	ch = USART_ReceiveData(USART2);
	    	ch = USART_ReceiveData(USART2);
	    	ch = USART_ReceiveData(USART2);
	    	//printf("%c", ch&0xff);

	    	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	    	break;
		}
  }

  USART_puts(USART2, ch);
}

// this is the interrupt request 	 (IRQ) for ALL USART1 interrupts
void USART2_IRQHandler(void){

	GPIO_SetBits(GPIOD, GPIO_Pin_14); // Toggle the LED, so you can see it change as you step each function
	//GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE)){

		GPIO_SetBits(GPIOD, GPIO_Pin_15); // Toggle the LED, so you can see it change as you step each function
		//GPIO_ResetBits(GPIOD, GPIO_Pin_15);

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			USART_puts(USART2, received_string);
		}
	}
}
