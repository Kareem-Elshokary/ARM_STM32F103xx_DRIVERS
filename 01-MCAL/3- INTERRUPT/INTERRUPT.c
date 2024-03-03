/*************************************************************************/
/* Author        : Kareem Abdelkader                                   			  	  	*/
/* Project       : STM32F103C8_Drivers  	                             			  	*/
/* File          : INTERRUPT.c		     		                     				    */
/* Date          : 22/2/2024                                        				  	*/
/* Version       : V2                                                				    */
/* GitHub        : https://github.com/Kareem-Elshokary/Master_Embedded_systems          */
/*************************************************************************/


/*****************************
 * Includes
 ****************************/
#include "INTERRUPT.h"

//========================================================================

/*****************************
 * Generic Variables
 ****************************/

void(* GP_IRQ_Callback[15])(void);




/*****************************
 * Generic Function
 ****************************/

// Enable EXTIx IRQ NVIC
void EXTI_IRQ_EN(NVIC_REG_t* NVICx, uint8_t EXTI_NUM)
{
	switch(EXTI_NUM)
	{
	case 0:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI0_);
	}
	break;
	case 1:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI1_);
	}
	break;
	case 2:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI2_);
	}
	break;
	case 3:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI3_);
	}
	break;
	case 4:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI4_);
	}
	break;
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		NVIC_IRQ_EN(NVICx->NVIC_ISER[0], EXTI9_5);
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		NVIC_IRQ_EN(NVICx->NVIC_ISER[1], (EXTI10_15 - 32));
		break;
	}
}



// Disable EXTIx IRQ NVIC
void EXTI_IRQ_DIS(NVIC_REG_t* NVICx, uint8_t EXTI_NUM)
{
	switch(EXTI_NUM)
	{
	case 0:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI0_);
	}
	break;
	case 1:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI1_);
	}
	break;
	case 2:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI2_);
	}
	break;
	case 3:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI3_);
	}
	break;
	case 4:
	{
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI4_);
	}
	break;
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		NVIC_IRQ_EN(NVICx->NVIC_ICER[0], EXTI9_5);
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		NVIC_IRQ_EN(NVICx->NVIC_ICER[1], (EXTI10_15 - 32));
		break;
	}
}



//=========================================================================

/**================================================================
 * @Fn			- MCAL_EXTI_INIT
 * @brief 		- Initialize EXTI from specific GPIO Pin and specify the mask\trigger condition and IRQ callback
 * @param [in]  - GPIOxx: GPIOx
 * @param [in]  - pin: pin number
 * @param [in]  - trigger_case: trigger case falling or rising
 * @param [in]  -func_add : app ISR function
 * @retval 		- none
 * Note			- none
 */
void MCAL_EXTI_INIT(GPIO_REG_t* GPIOxx, uint16_t pin, uint32_t trigger_case, void(* func_add) (void))
{
	MCAL_GPIO_INIT(GPIOxx, pin, INPUT_AF, 0);

	switch(pin)
	{
	case PIN0:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR1 &= ~(0b1111 << 0); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR1 |= (0b0001 << 0); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR1 |= (0b0010 << 0); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR1 |= (0b0011 << 0); }
	}
	break;
	case PIN1:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR1 &= ~(0b1111 << 4); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR1 |= (0b0001 << 4); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR1 |= (0b0010 << 4); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR1 |= (0b0011 << 4); }
	}
	break;
	case PIN2:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR1 &= ~(0b1111 << 8); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR1 |= (0b0001 << 8); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR1 |= (0b0010 << 8); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR1 |= (0b0011 << 8); }
	}
	break;
	case PIN3:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR1 &= ~(0b1111 << 12); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR1 |= (0b0001 << 12); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR1 |= (0b0010 << 12); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR1 |= (0b0011 << 12); }
	}
	break;
	case PIN4:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR2 &= ~(0b1111 << 0); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR2 |= (0b0001 << 0); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR2 |= (0b0010 << 0); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR2 |= (0b0011 << 0); }
	}
	break;
	case PIN5:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR2 &= ~(0b1111 << 4); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR2 |= (0b0001 << 4); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR2 |= (0b0010 << 4); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR2 |= (0b0011 << 4); }
	}
	break;
	case PIN6:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR2 &= ~(0b1111 << 8); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR2 |= (0b0001 << 8); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR2 |= (0b0010 << 8); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR2 |= (0b0011 << 8); }
	}
	break;
	case PIN7:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR2 &= ~(0b1111 << 12); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR2 |= (0b0001 << 12); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR2 |= (0b0010 << 12); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR2 |= (0b0011 << 12); }
	}
	break;
	case PIN8:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR3 &= ~(0b1111 << 0); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR3 |= (0b0001 << 0); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR3 |= (0b0010 << 0); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR3 |= (0b0011 << 0); }
	}
	break;
	case PIN9:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR3 &= ~(0b1111 << 4); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR3 |= (0b0001 << 4); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR3 |= (0b0010 << 4); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR3 |= (0b0011 << 4); }
	}
	break;
	case PIN10:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR3 &= ~(0b1111 << 8); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR3 |= (0b0001 << 8); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR3 |= (0b0010 << 8); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR3 |= (0b0011 << 8); }
	}
	break;
	case PIN11:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR3 &= ~(0b1111 << 12); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR3 |= (0b0001 << 12); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR3 |= (0b0010 << 12); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR3 |= (0b0011 << 12); }
	}
	break;
	case PIN12:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR4 &= ~(0b1111 << 0); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR4 |= (0b0001 << 0); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR4 |= (0b0010 << 0); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR4 |= (0b0011 << 0); }
	}
	break;
	case PIN13:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR4 &= ~(0b1111 << 4); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR4 |= (0b0001 << 4); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR4 |= (0b0010 << 4); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR4 |= (0b0011 << 4); }
	}
	break;
	case PIN14:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR4 &= ~(0b1111 << 8); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR4 |= (0b0001 << 8); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR4 |= (0b0010 << 8); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR4 |= (0b0011 << 8); }
	}
	break;
	case PIN15:
	{
		if(GPIOxx == GPIOA){ AFIO->AFIO_EXTICR4 &= ~(0b1111 << 12); }
		else if(GPIOxx == GPIOB){ AFIO->AFIO_EXTICR4 |= (0b0001 << 12); }
		else if(GPIOxx == GPIOC){ AFIO->AFIO_EXTICR4 |= (0b0010 << 12); }
		else if(GPIOxx == GPIOD){ AFIO->AFIO_EXTICR4 |= (0b0011 << 12); }
	}
	break;
	}

	switch(trigger_case)
	{
	case RISING_EDGE:
	{
		EXTI->EXTI_RTSR &= ~(1 << pin);
		EXTI->EXTI_RTSR |= (1<<pin);
	}
	break;
	case FALLING_EDGE:
	{
		EXTI->EXTI_FTSR &= ~(1 << pin);
		EXTI->EXTI_FTSR |= (1<<pin);
	}
	break;
	case RISING_FALLING_EDGE:
	{
		EXTI->EXTI_RTSR &= ~(1 << pin);
		EXTI->EXTI_FTSR &= ~(1 << pin);
		EXTI->EXTI_RTSR |= (1<<pin);
		EXTI->EXTI_FTSR |= (1<<pin);
	}
	break;
	}

	EXTI->EXTI_IMR |= (1<<pin);

	GP_IRQ_Callback[pin] = func_add;

	EXTI_IRQ_EN(NVIC, pin);

}


/**================================================================
 * @Fn			- MCAL_GPIO_DEINIT
 * @brief 		- Reset the GPIOx PINy according to a specific parameter in the Pin Configuration
 * @param [in] 	- GPIOx: x can be (A:G depend on the device used) to select the GPIO peripheral
 * @retval 		- none
 * Note			- Can be reset by reset controller
 */
void MCAL_EXTI_DEINIT(AFIO_REG_t* AFIOx)
{
	AFIOx->AFIO_EXTICR1 = 0x0000;
	AFIOx->AFIO_EXTICR2 = 0x0000;
	AFIOx->AFIO_EXTICR3 = 0x0000;
	AFIOx->AFIO_EXTICR4 = 0x0000;
}



void EXTI0_IRQHandler(void){
	// clear bit in pending register (EXTI_PR)
	EXTI->EXTI_PR |= (1<<0);
	GP_IRQ_Callback[0]();
}

void EXTI1_IRQHandler(void){
	EXTI->EXTI_PR |= (1<<1);
	GP_IRQ_Callback[1]();

}

void EXTI2_IRQHandler(void){
	EXTI->EXTI_PR |= (1<<2);
	GP_IRQ_Callback[2]();
}

void EXTI3_IRQHandler(void){
	EXTI->EXTI_PR |= (1<<3);
	GP_IRQ_Callback[3]();
}

void EXTI4_IRQHandler(void){
	EXTI->EXTI_PR |= (1<<4);
	GP_IRQ_Callback[4]();
}

void EXTI9_5_IRQHandler(void){
	if(EXTI->EXTI_PR & (1<<5)) {EXTI->EXTI_PR |= (1<<5); GP_IRQ_Callback[5]();  }
	if(EXTI->EXTI_PR & (1<<6)) {EXTI->EXTI_PR |= (1<<6); GP_IRQ_Callback[6]();  }
	if(EXTI->EXTI_PR & (1<<7)) {EXTI->EXTI_PR |= (1<<7); GP_IRQ_Callback[7]();  }
	if(EXTI->EXTI_PR & (1<<8)) {EXTI->EXTI_PR |= (1<<8); GP_IRQ_Callback[8]();  }
	if(EXTI->EXTI_PR & (1<<9)) {EXTI->EXTI_PR |= (1<<9); GP_IRQ_Callback[9]();  }
}

void EXTI15_10_IRQHandler(void){
	if(EXTI->EXTI_PR & (1<<10)) {EXTI->EXTI_PR |= (1<<10); GP_IRQ_Callback[10]();  }
	if(EXTI->EXTI_PR & (1<<11)) {EXTI->EXTI_PR |= (1<<11); GP_IRQ_Callback[11]();  }
	if(EXTI->EXTI_PR & (1<<12)) {EXTI->EXTI_PR |= (1<<12); GP_IRQ_Callback[12]();  }
	if(EXTI->EXTI_PR & (1<<13)) {EXTI->EXTI_PR |= (1<<13); GP_IRQ_Callback[13]();  }
	if(EXTI->EXTI_PR & (1<<14)) {EXTI->EXTI_PR |= (1<<14); GP_IRQ_Callback[14]();  }
	if(EXTI->EXTI_PR & (1<<15)) {EXTI->EXTI_PR |= (1<<15); GP_IRQ_Callback[15]();  }
}
