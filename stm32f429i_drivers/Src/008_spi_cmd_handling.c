/*
 * 008_spi_cmd_handling.c
 *
 *  Created on: 2022年7月1日
 *      Author: lin
 */


#include <string.h>
#include "stm32f429i.h"


// command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON     1
#define LED_OFF    0

// arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

// arduino led
#define LED_PIN 9

void delay(void) {
	for (uint32_t i = 0; i < 1000000; i++);
}


/*
 * ALT function mode: 5
 * PA5->SPI1_SCK
 * PA7->SPI1_MOSI
 * PA6->SPI1_MISO
 * PA4->SPI1_NSS
 */
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

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

uint8_t SPI_VerifyResponse(uint8_t ACKByte) {

	if (ACKByte == 0xF5) {
		return 1;
	}

	return 0;

}


int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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

        // COMMAND_LED_CTRL

		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		// Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		// 1. CMD_LED_CTRL <pin no(1)>   <value(1)>
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ACKByte;
		uint8_t args[2];

		// Send command
		SPI_SendData(SPI1, &commandCode, 1);

		// Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		// Send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI1, &dummy_write, 1);

		// Read the ack byte received
		SPI_ReceiveData(SPI1, &ACKByte, 1);

		if (SPI_VerifyResponse(ACKByte)) {

			// Send arguments;
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI1, args, 2);

		}


		// COMMAND_SENSOR_READ

		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		// 1. CMD_LED_CTRL <pin no(1)>   <value(1)>
		commandCode = COMMAND_SENSOR_READ;

		// Send command
		SPI_SendData(SPI1, &commandCode, 1);

		// Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		// Send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI1, &dummy_write, 1);

		// Read the ack byte received
		SPI_ReceiveData(SPI1, &ACKByte, 1);

		if (SPI_VerifyResponse(ACKByte)) {

			// Send arguments;
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI1, args, 1);

			// Do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1, &dummy_read, 1);

			delay();

			// Send some dummy bits (1byte) to fetch the response from the slave.
			SPI_SendData(SPI1, &dummy_write, 1);

			// Do dummy read to clear off the RXNE
			uint8_t analog_read;
			SPI_ReceiveData(SPI1, &analog_read, 1);

		}

        // COMMAND_LED_READ

        while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay();

        // 1. CMD_LED_CTRL <pin no(1)>   <value(1)>
        commandCode = COMMAND_LED_READ;

        // Send command
        SPI_SendData(SPI1, &commandCode, 1);

        // Do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Send some dummy bits (1byte) to fetch the response from the slave.
        SPI_SendData(SPI1, &dummy_write, 1);

        // Read the ack byte received
        SPI_ReceiveData(SPI1, &ACKByte, 1);

        if (SPI_VerifyResponse(ACKByte)) {

            // Send arguments;
            args[0] = LED_PIN;
            SPI_SendData(SPI1, args, 1);

            // Do dummy read to clear off the RXNE
            SPI_ReceiveData(SPI1, &dummy_read, 1);

            delay();

            // Send some dummy bits (1byte) to fetch the response from the slave.
            SPI_SendData(SPI1, &dummy_write, 1);

            // Do dummy read to clear off the RXNE
            uint8_t led_status;
            SPI_ReceiveData(SPI1, &led_status, 1);

        }

        // COMMAND_PRINT

        while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay();

        // 1. CMD_LED_CTRL <pin no(1)>   <value(1)>
        commandCode = COMMAND_PRINT;

        // Send command
        SPI_SendData(SPI1, &commandCode, 1);

        // Do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Send some dummy bits (1byte) to fetch the response from the slave.
        SPI_SendData(SPI1, &dummy_write, 1);

        // Read the ack byte received
        SPI_ReceiveData(SPI1, &ACKByte, 1);

        if (SPI_VerifyResponse(ACKByte)) {

            uint8_t message[] = "Hello world2";

            // Send arguments;
            args[0] = strlen((char*) message);
            SPI_SendData(SPI1, args, 1);


            // Do dummy read to clear off the RXNE
            SPI_ReceiveData(SPI1, &dummy_read, 1);

            // Send message
            for (int i = 0; i < args[0]; i++) {
                SPI_SendData(SPI1, &message[i], 1);
                SPI_ReceiveData(SPI1, &dummy_read, 1);
            }

        }

        // COMMAND_ID_READ

        while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay();

        // 1. CMD_LED_CTRL <pin no(1)>   <value(1)>
        commandCode = COMMAND_ID_READ;

        // Send command
        SPI_SendData(SPI1, &commandCode, 1);

        // Do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Send some dummy bits (1byte) to fetch the response from the slave.
        SPI_SendData(SPI1, &dummy_write, 1);

        // Read the ack byte received
        SPI_ReceiveData(SPI1, &ACKByte, 1);


        uint8_t id[11];
        if (SPI_VerifyResponse(ACKByte)) {

            //read 10 bytes id from the slave
            for(uint32_t i = 0 ; i < 10 ; i++)
            {
                //send dummy byte to fetch data from slave
                SPI_SendData(SPI1,&dummy_write,1);
                SPI_ReceiveData(SPI1,&id[i],1);
            }

            id[10] = '\0';

        }


        // let's confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

		// Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);

    }

}
