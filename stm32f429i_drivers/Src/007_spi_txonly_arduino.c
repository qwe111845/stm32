/*
 * 007_spi_txonly_arduino.c
 *
 *  Created on: 2022年6月29日
 *      Author: lin
 */


#include <string.h>
#include "stm32f429i.h"

/*
 * ALT function mode: 5
 * PA5->SPI1_SCK
 * PA7->SPI1_MOSI
 * PA6->SPI1_MISO
 * PA4->SPI1_NSS
 */


void delay(void) {
	for (uint32_t i = 0; i < 1000000; i++);
}
void SPI_GPIOInits(void) {

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIO = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode  = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	// MISO
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	// GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

}

void SPI1_Inits(void) {

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPI = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8; // generated sclk of 8MHz
    SPI1handle.SPIConfig.SPI_DFF        = SPI_DFF_8_BITS;
    SPI1handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
    SPI1handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
    SPI1handle.SPIConfig.SPI_SSM        = SPI_SSM_DISABLE; // Hardware software slave management enabled for NSS pin

    SPI_Init(&SPI1handle);

}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIO = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_INPUT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOButton);

}

int main(void)
{

	char user_data[] = "Hi. Where is 001SPISlaveRxString.form me or make available in the resources?";

	GPIO_ButtonInit();

	// This function is used to initialize the GPIO pins to behave as SPI1 pin
	SPI_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI1_Inits();

	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by the hard way.
	 * i.e when SPE=1, NSS will be pulled by pull
	 */
    SPI_SSOEConfig(SPI1, ENABLE);

    while(1) {

		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		// Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		// first send length information
		uint8_t dataLength = strlen(user_data);
		SPI_SendData(SPI1, &dataLength, 1);



		// To send data
		SPI_SendData(SPI1, (uint8_t *)user_data, dataLength);

		// lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

		// Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);

    }

	return 0;
}
