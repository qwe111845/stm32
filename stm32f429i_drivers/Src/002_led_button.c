#include <stdint.h>
#include "stm32f429i.h"

#define HIGH        ENABLE
#define BTN_PRESSED HIGH

void delay(void) {
	for (uint32_t i = 0; i < 300000; i++);
}

int main(void)
{


	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIO = GPIOG;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClocckControl(GPIOG, ENABLE);

	GPIO_Init(&GPIOLed);

	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIO = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_INPUT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClocckControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOButton);


	while (1) {
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
		}
	}


	return 0;
}
