//
// Created by qwe111845 on 2022/7/17.
//
/*
 * Peripheral Clock setup
 */
#include "stm32f429zi_i2c_driver.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi) {
    if (enOrDi == ENABLE) {

        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }


    } else {

        if (pI2Cx == I2C1) {
            I2C1_PCLK_DIS();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DIS();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DIS();
        }

    }
}

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

    if (pI2Cx ==  I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx ==  I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx ==  I2C3) {
        I2C3_REG_RESET();
    }

}


/*
 * Data Send and Receive
 */
void I2C_SendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t length);
void I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t length);


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi) {

    // 根據 NVIC 的 vector table 找到最多實作了多少 interrupt (stm32f429xx 為 90)
    if (enOrDi == ENABLE) {
        if (IRQNumber <= 31) {

            // program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber <= 64) {

            // program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber > 64 && IRQNumber <= 96) {

            // program ISER2 register
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));

        }
    } else {

        if (IRQNumber <= 31) {

            // program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber <= 64) {

            // program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber > 64 && IRQNumber <= 96) {

            // program ICER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));

        }
    }

}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

    // 1. first lets find out the IPR register
    uint8_t IPRx = IRQNumber / 4;
    uint8_t IPRxSection = IRQNumber % 4;

    // IPR 每個 PRI 的前四bit 都沒有作用 所以要 8-PR BITS
    uint8_t shift_amount = (8 * IPRxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

    // IRQPriority 0-255 在 Cortex-M4 Devices Generic User Guide 223頁 IPR
    *(NVIC_PR_BASE_ADDR + IPRx) |= (IRQPriority << shift_amount);

}
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi) {

    if (enOrDi == ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }

}
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
void I2C_SSOEConfig(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);