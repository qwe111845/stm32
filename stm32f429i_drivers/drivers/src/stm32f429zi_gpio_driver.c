/*
 * stm32f429zi_gpio_driver.c
 *
 *  Created on: 2022年5月20日
 *      Author: qwe111845
 */

#include "stm32f429zi_gpio_driver.h"


/*
 * Peripheral Clock setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClocckControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ) {
            GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK) {
            GPIOK_PCLK_EN();
		}

	} else {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DIS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DIS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DIS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DIS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DIS();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DIS();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DIS();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DIS();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DIS();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DIS();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_DIS();
		}
	}

}


/*
 * Init and De-init
 */
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialize peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral and GPIO pin configuration
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIO, ENABLE);

	// 1. configure the mode of GPIO pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIO->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIO->MODER |= temp;

	} else {

		// the interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

			// 1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

			// 1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {

			// 1. configure both FTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t EXTICRIndex    = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t EXTICRPosition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[EXTICRIndex] = portCode << (EXTICRPosition * 4);


		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;


	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->OSPEEDR |= temp;

	temp = 0;


	// 3. configure the pull-up or pull-down setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->PUPDR |= temp;

	temp = 0;

	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER |= temp;

	temp = 0;

	// 5. configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure the alternate function registers
		uint32_t AFRIndex, AFRPinPosition;
		AFRIndex       = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		AFRPinPosition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIO->AFR[AFRIndex] &= ~(0xf << (4 * AFRPinPosition));
		pGPIOHandle->pGPIO->AFR[AFRIndex] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * AFRPinPosition));
	}

}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function De-initialize peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx ==  GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx ==  GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx ==  GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx ==  GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx ==  GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx ==  (GPIOF)) {
		GPIOF_REG_RESET();
	} else if (pGPIOx ==  GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx ==  GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx ==  GPIOI) {
		GPIOI_REG_RESET();
	} else if (pGPIOx ==  GPIOJ) {
		GPIOJ_REG_RESET();
	} else if (pGPIOx ==  GPIOK) {
		GPIOK_REG_RESET();
	}

}


/*
 * Data read and write
 */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads data from given GPIO pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - GPIO pin number
 * @param[in]         -
 *
 * @return            - 0-1
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR  >> pinNumber) & 0x00000001);

	return value;

}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads data from given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 16 bit data
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes data to given GPIO ping
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - GPIO pin number
 * @param[in]         - input value (1 bit)
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {

	if (value == GPIO_PIN_SET) {

		// Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << pinNumber);

	} else {

		// Write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << pinNumber);

	}

}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes data to given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - input value
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

	pGPIOx->ODR = value;

}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function changes output GPIO pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - GPIO pin number
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	pGPIOx->ODR ^= (1 << pinNumber);

}


/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function changes configuration of IRQ
 *
 * @param[in]         - IRQ number
 * @param[in]         - enable or disable interrupt
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi) {

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


/*********************************************************************
 * @fn      		  - GPIO_IRQPriortyConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	// 1. first lets find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRxSection = IRQNumber % 4;

    // IPR 每個 PRI 的前四bit 都沒有作用 所以要 8-PR BITS
	uint8_t shift_amount = (8 * IPRxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

    // IRQPriority 0-255 在 Cortex-M4 Devices Generic User Guide 223頁 IPR
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQPriority << shift_amount);

}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function manages the interrupt of GPIO pin
 *
 * @param[in]         - GPIO pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t pinNumber) {

	// clear the EXTI pending-register register corresponding to the pin number

	if (EXTI->PR & (1 << pinNumber)) {
		// clear
		EXTI->PR |= (1 << pinNumber);
	}

}
