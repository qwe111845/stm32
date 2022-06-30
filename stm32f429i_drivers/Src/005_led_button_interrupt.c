#include <stdint.h>
#include <string.h>
#include "stm32f429i.h"

#define HIGH        ENABLE
#define LOW         DISABLE

#define BTN_PRESSED LOW

void delay(void) {
	for (uint32_t i = 0; i < 300000; i++);
}

int main(void)
{


	GPIO_Handle_t GPIOLed;
	memset(&GPIOLed, 0, sizeof(GPIOLed));

	GPIOLed.pGPIO = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_LOW;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClocckControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

	GPIO_Handle_t GPIOButton;
	memset(&GPIOButton, 0, sizeof(GPIOButton));

	GPIOButton.pGPIO = GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClocckControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOButton);


	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

}


void EXTI9_5_IRQHandler(void) {

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}
