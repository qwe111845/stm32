//
// Created by qwe111845 on 2022/7/17.
//
/*
 * Peripheral Clock setup
 */
#include "stm32f429zi_i2c_driver.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[8] = { 2, 4, 8, 16};

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

// TODO
uint32_t RCC_GetPLLOutputClock(void) {

}

uint32_t RCC_GetPCLK1Value(void) {
    uint32_t  pClock, systemClock = 0;
    uint8_t  clockSource, temp, ahbp, apb1p;

    clockSource = (RCC->CFGR >> 2) & 0x3;

    if (clockSource == 0) {
        systemClock = 16000000;
    } else if (clockSource == 1) {
        systemClock = 8000000;
    } else if (clockSource == 2) {
        systemClock = RCC_GetPLLOutputClock();
    }

    // ahb
    temp = ((RCC->CFGR >> 4) & 0xF);

    if (temp < 8) {
        ahbp = 1;
    } else {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // apb1
    temp = ((RCC->CFGR >> 10) & 0x7);

    if (temp < 4) {
        apb1p = 1;
    } else {
        apb1p = APB1_PreScaler[temp - 4];
    }
    pClock = (systemClock / ahbp) / apb1p;

    return  pClock;

}
/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempReg = 0;

    // ack control bit
    tempReg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;

    pI2CHandle->pI2C->CR1 = tempReg;

    // configure the FREQ field of CR2
    tempReg = 0;
    tempReg = RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2C->CR2 = (tempReg & 0x3F);

    // program the device own address
    tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempReg |= (1 << 14);
    pI2CHandle->pI2C->OAR1 = tempReg;

    // CCR calculations
    uint16_t  ccr_value = 0;
    tempReg = 0;

    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {

        // mode is standard mode
        ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        tempReg |= ccr_value & 0xFFF;

    } else {

        // mode is fast mode
        tempReg |= (1 << 15);
        tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        } else {
            ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

        }

        tempReg |= (ccr_value & 0xFFF);

    }

    pI2CHandle->pI2C->CCR = tempReg;

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