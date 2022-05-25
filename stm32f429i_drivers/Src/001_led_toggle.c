#include <stdint.h>
#include "stm32f429i.h"

void delay(void) {
	for (uint32_t i = 0; i < 100000; i++);
}

int main(void)
{



	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIO = GPIOG;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClocckControl(GPIOG, ENABLE);

	GPIO_Init(&GPIOLed);


	while (1) {
		GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);

		delay();
	}


	return 0;
}
