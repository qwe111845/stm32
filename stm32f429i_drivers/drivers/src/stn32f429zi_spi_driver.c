/*
 * stn32f429zi_spi_driver.c
 *
 *  Created on: 2022年6月24日
 *      Author: lin
 */

#include "stm32f429zi_spi_driver.h"


/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {

		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_EN();
		} else if (pSPIx == SPI6) {
			SPI6_PCLK_EN();
		}

	} else {

		if (pSPIx == SPI1) {
			SPI1_PCLK_DIS();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DIS();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DIS();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DIS();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_DIS();
		} else if (pSPIx == SPI6) {
			SPI6_PCLK_DIS();
		}

	}

}

/*
 * Init and De-init
 */

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initialize peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral and SPI pin configuration
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPI, ENABLE);

	// first  configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {

		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {

		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		// READONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	// 3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the SPI DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the SPI CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the SPI CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the SPI SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPI->CR1 = tempreg;

}


/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function De-initialize peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	if (pSPIx ==  SPI1) {
			SPI1_REG_RESET();
		} else if (pSPIx ==  SPI2) {
			SPI2_REG_RESET();
		} else if (pSPIx ==  SPI3) {
			SPI3_REG_RESET();
		} else if (pSPIx ==  SPI4) {
			SPI4_REG_RESET();
		} else if (pSPIx ==  SPI5) {
			SPI5_REG_RESET();
		} else if (pSPIx ==  (SPI6)) {
			SPI6_REG_RESET();
		}

}



/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function gets status from status register by given FlagName
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - position of SR flag
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {

	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data from given SPI
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - data to send
 * @param[in]         - length of data
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length) {

    // Check data length is greater than zero
	while (length > 0) {

		// 1. Wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {

			// 16 bit DFF
			// 1. load the data in to the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			length--;
			length--;
			(uint16_t *)pTxBuffer++;

		} else {

			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;

		}

	}
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length) {

	// Check data length is greater than zero
		while (length > 0) {

			// 1. Wait until RXNE is set
			while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			// 2. Check the DFF bit in CR1
			if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {

				// 16 bit DFF
				// 1. load the data from DR to Rxbuffer address
				*((uint16_t *)pRxBuffer) = pSPIx->DR;
				length--;
				length--;
				(uint16_t *)pRxBuffer++;

			} else {

				// 8 bit DFF
				*(pRxBuffer) = pSPIx->DR;
				length--;
				pRxBuffer++;

			}

		}

}


/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);


/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */


/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables or disables serial peripheral for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function enables or disables internal slave select for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - This function enables or disables slave select output for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi) {

    if (enOrDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }

}

