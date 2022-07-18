//
// Created by qwe111845 on 2022/7/17.
//

#ifndef STM32F429I_DRIVERS_STM32F429ZI_I2C_DRIVER_H
#define STM32F429I_DRIVERS_STM32F429ZI_I2C_DRIVER_H

#include "stm32f429i.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
    uint32_t I2C_SCLSpeed;   // MSTR possible values from @I2C_DeviceMode
    uint8_t  I2C_DeviceAddress;     // BIDIMODE possible values from @I2C_BusConfig
    uint8_t  I2C_AckControl;     // BR[2:0] possible values from @I2C_SclkSpeed
    uint8_t  I2C_FMDutyCycle;          // DFF possible values from @I2C_DFF
} I2C_Config_t;


/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
    I2C_RegDef_t *pI2C;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM   100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE  1
#define I2C_ACK_DISABLE 0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2    0
#define I2C_FM_DUTY_16_9 1


/********************************************************************************************
 *                             APIs supported by this driver
 *             For more information about the APIs check the function definitions
 ********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_SendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t length);
void I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t length);


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
void I2C_SSOEConfig(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

#endif //STM32F429I_DRIVERS_STM32F429ZI_I2C_DRIVER_H
