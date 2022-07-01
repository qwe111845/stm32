/*
 * 006_spi_tx_testing.c
 *
 *  Created on: 2022年6月27日
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
	for (uint32_t i = 0; i < 100000; i++);
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
	SPI1handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV2; // generated sclk of 8MHz
    SPI1handle.SPIConfig.SPI_DFF        = SPI_DFF_8_BITS;
    SPI1handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
    SPI1handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
    SPI1handle.SPIConfig.SPI_SSM        = SPI_SSM_ENABLE; // software slave management enabled for NSS pin

    SPI_Init(&SPI1handle);

}

int main(void)
{

	char user_data[] = "Hello world";

	// This function is used to initialize the GPIO pins to behave as SPI1 pin
	SPI_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI1_Inits();

	// This makes NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	// Enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, ENABLE);

	// To send data
	SPI_SendData(SPI1, (uint8_t *)user_data, strlen(user_data));

	// lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

	// Enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, DISABLE);


	return 0;
}
