/*
 * stn32f429zi_spi_driver.c
 *
 *  Created on: 2022年6月24日
 *      Author: lin
 */

#include "stm32f429zi_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function return status from status register by given FlagName
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - position of SR flag
 * @param[in]         -
 *
 * @return            - Flag status
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
 * @brief             - This function sends data to given SPI MOSI pin
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - data to be transmitted
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
 * @brief             - This function reads data from given SPI MISO pin
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - received data
 * @param[in]         - length of data
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
 * @brief             - This function changes configuration of IRQ ISER
 *
 * @param[in]         - IRQNumber
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi)
{

    if(enOrDi == ENABLE) {

        if(IRQNumber <= 31) {

            //program ISER0 register
            *NVIC_ISER0 |= ( 1 << IRQNumber );

        } else if(IRQNumber > 31 && IRQNumber < 64 )  { //32 to 63

            //program ISER1 register
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

        } else if(IRQNumber >= 64 && IRQNumber < 96 ) {

            //program ISER2 register //64 to 95
            *NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );

        }

    } else {

        if(IRQNumber <= 31) {

            //program ICER0 register
            *NVIC_ICER0 |= ( 1 << IRQNumber );

        } else if(IRQNumber > 31 && IRQNumber < 64 ) {

            //program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));

        } else if(IRQNumber >= 6 && IRQNumber < 96 ) {

            //program ICER2 register
            *NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );

        }

    }

}


/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function changes priority of IRQ
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
    //1. first lets find out the ipr register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber % 4 ;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

    *(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}


/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - API to send data with interrupt mode
 *
 * @param[in]         - base address of the SPI peripheral and SPI configuration
 * @param[in]         - data to be transmitted
 * @param[in]         - length of data
 *
 * @return            - Tx State
 *
 * @Note              - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length) {

    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX) {

        // 1. Save the Tx buffer address and length information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLength = length;

        // 2. Mark the SPI state as busy in transmission so that
        //    no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_TXNEIE);


    }

    // 4. Data Transmission will be handled by the ISR code ( will implement later)
    return state;
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - API to receive data with interrupt mode
 *
 * @param[in]         - base address of the SPI peripheral and SPI configuration
 * @param[in]         - data to be received
 * @param[in]         - length of data
 *
 * @return            - Rx State
 *
 * @Note              - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length) {

    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX) {

        // 1. Save the Tx buffer address and length information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLength = length;

        // 2. Mark the SPI state as busy in transmission so that
        //    no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_RXNEIE);

    }

    return state;

}

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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

    uint8_t error, needToHandle;

    // check for TXE
    error = pSPIHandle->pSPI->SR & (1 << SPI_SR_TXE);
    needToHandle = pSPIHandle->pSPI->CR2 & (1 << SPI_CR2_TXNEIE);

    if (error & needToHandle) {

        // handle TXE
        spi_txe_interrupt_handle(pSPIHandle);

    }

    // check for RXE
    error = pSPIHandle->pSPI->SR & (1 << SPI_SR_RXNE);
    needToHandle = pSPIHandle->pSPI->CR2 & (1 << SPI_CR2_RXNEIE);

    if (error & needToHandle) {

        // handle RXE
        spi_rxne_interrupt_handle(pSPIHandle);

    }

    // check for OVR flag
    error = pSPIHandle->pSPI->SR & (1 << SPI_SR_RXNE);
    needToHandle = pSPIHandle->pSPI->CR2 & (1 << SPI_CR2_RXNEIE);

    if (error & needToHandle) {

        // handle OVR
        spi_ovr_err_interrupt_handle(pSPIHandle);

    }

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    // 2. Check the DFF bit in CR1
    if ((pSPIHandle->pSPI->CR1 & (1 << SPI_CR1_DFF))) {

        // 16 bit DFF
        // 1. load the data in to the DR
        pSPIHandle->pSPI->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLength--;
        pSPIHandle->TxLength--;
        (uint16_t *)pSPIHandle->pTxBuffer++;

    } else {

        // 8 bit DFF
        pSPIHandle->pSPI->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLength--;
        pSPIHandle->pTxBuffer++;

    }

    if (!pSPIHandle->TxLength) {

        CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }

}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    // 2. Check the DFF bit in CR1
    if ((pSPIHandle->pSPI->CR1 & (1 << SPI_CR1_DFF))) {

        // 16 bit DFF
        // 1. load the data in to the DR
        *((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPI->DR;
        pSPIHandle->RxLength -= 2;
        (uint16_t *)pSPIHandle->pRxBuffer--;
        (uint16_t *)pSPIHandle->pRxBuffer--;

    } else {

        // 8 bit DFF
        *pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPI->DR;
        pSPIHandle->RxLength--;
        pSPIHandle->pRxBuffer--;

    }

    if (!pSPIHandle->RxLength) {

        CloseReception(pSPIHandle);

        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    uint8_t temp;

    // 1. clear the ovr flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        temp = pSPIHandle->pSPI->DR;
        temp = pSPIHandle->pSPI->SR;
    }
    (void) temp;

    // 2. inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);


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
void CloseTransmission(SPI_Handle_t *pSPIHandle) {
    // TxLength is zero, so close the spi transmission and inform the application that
    // Tx is over

    // this prevents interrupts from setting up of TXE flag
    pSPIHandle->pSPI->CR2 &= ~(1 << SPI_CR2_TXNEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLength = 0;
    pSPIHandle->TxState = SPI_READY;
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
void CloseReception(SPI_Handle_t *pSPIHandle) {

    // RxLength is zero, so close the spi transmission and inform the application that
    // Rx is over

    // this prevents interrupts from setting up of RXE flag
    pSPIHandle->pSPI->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->RxLength = 0;
    pSPIHandle->RxState = SPI_READY;

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
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

    uint8_t temp;

    // 1. clear the ovr flag
    temp = pSPIx->DR;
    temp = pSPIx->SR;

    (void) temp;

}





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


/*********************************************************************
 * @fn      		  - SPI_ApplicationEventCallback
 *
 * @brief             - This function enables or disables slave select output for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral and SPI config
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {

    // This is a weak implementation. the user application may override this function

}
