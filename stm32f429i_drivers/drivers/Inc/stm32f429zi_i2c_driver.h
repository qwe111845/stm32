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
    uint32_t I2C_SCLSpeed;          // SCL speed possible values from @I2C_SCLSpeed
    uint8_t  I2C_DeviceAddress;     // Slave device address
    uint8_t  I2C_AckControl;        // possible values from @I2C_AckControl
    uint8_t  I2C_FMDutyCycle;       // possible values from @I2C_FMDutyCycle
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

/*
 * @I2C read write bit
 */
#define I2C_READ     0
#define I2C_WRITE    1

/*
 * @I2C related status flags definitions
 */
#define I2C_FLAG_SB          (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR        (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF         (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF       (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE        (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE         (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR        (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO        (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF          (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR         (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR      (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIME_OUT    (1 << I2C_SR1_TIME_OUT)
#define I2C_FLAG_SMB_ALERT   (1 << I2C_SR1_SMB_ALERT)


#define I2C_ENABLE_SR     SET
#define I2C_DISABLE_SR  RESET

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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t SR);


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
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
#endif //STM32F429I_DRIVERS_STM32F429ZI_I2C_DRIVER_H
