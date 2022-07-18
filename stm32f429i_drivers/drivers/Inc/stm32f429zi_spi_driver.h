/*
 * stm32f429zi_spi.h
 *
 *  Created on: 2022年6月24日
 *      Author: lin
 */

#ifndef INC_STM32F429ZI_SPI_DRIVER_H_
#define INC_STM32F429ZI_SPI_DRIVER_H_

#include "stm32f429i.h"

typedef struct {
	uint8_t SPI_DeviceMode;   // MSTR possible values from @SPI_DeviceMode
	uint8_t SPI_BusConfig;    // BIDIMODE possible values from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;    // BR[2:0] possible values from @SPI_SclkSpeed
	uint8_t SPI_DFF;          // DFF possible values from @SPI_DFF
	uint8_t SPI_CPOL;         // CPOL possible values from @SPI_CPOL
	uint8_t SPI_CPHA;         // CPHA possible values from @SPI_CPHA
	uint8_t SPI_SSM;          // SSM possible values from @SPI_SSM
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPI;      // This holds the base address of the SPI port to which the pin belongs
	SPI_Config_t SPIConfig;  // This holds SPI pin configuration setting
    uint8_t      *pTxBuffer; // To store the app. Tx buffer address
    uint8_t      *pRxBuffer; // To store the app. Rx buffer address
    uint32_t     TxLength;   // To store Tx length
    uint32_t     RxLength;   // To store Rx length
    uint8_t      TxState;    // To store Tx state
    uint8_t      RxState;    // To store Rx state
} SPI_Handle_t;


/*
 * SPI application state
 */
#define SPI_READY        0
#define SPI_BUSY_IN_RX   1
#define SPI_BUSY_IN_TX   2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD              1
#define SPI_BUS_CONFIG_HD              2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY  3


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8_BITS  0
#define SPI_DFF_16_BITS 1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0

/*
 * @SPI_SSM
 */
#define SPI_SSM_ENABLE   1
#define SPI_SSM_DISABLE  0

/*
 * @SPI related status flags definitions
 */
#define SPI_RXNE_FLAG     (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG      (1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG   (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG      (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG   (1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG     (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG      (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG      (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG      (1 << SPI_SR_FRE)




/********************************************************************************************
 *                             APIs supported by this driver
 *             For more information about the APIs check the function definitions
 ********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClocckControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void CloseTransmission(SPI_Handle_t *pSPIHandle);
void CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);







#endif /* INC_STM32F429ZI_SPI_DRIVER_H_ */
