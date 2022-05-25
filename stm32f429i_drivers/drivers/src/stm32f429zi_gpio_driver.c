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
void GPIO_PeriClocckControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLOCK_EN();
		} else if (pGPIOx == GPIOB) {
            GPIOB_PCLOCK_EN();
		} else if (pGPIOx == GPIOC) {
            GPIOC_PCLOCK_EN();
		} else if (pGPIOx == GPIOD) {
            GPIOD_PCLOCK_EN();
		} else if (pGPIOx == GPIOE) {
            GPIOE_PCLOCK_EN();
		} else if (pGPIOx == GPIOF) {
            GPIOF_PCLOCK_EN();
		} else if (pGPIOx == GPIOG) {
            GPIOG_PCLOCK_EN();
		} else if (pGPIOx == GPIOH) {
            GPIOH_PCLOCK_EN();
		} else if (pGPIOx == GPIOI) {
            GPIOI_PCLOCK_EN();
		} else if (pGPIOx == GPIOJ) {
            GPIOJ_PCLOCK_EN();
		} else if (pGPIOx == GPIOK) {
            GPIOK_PCLOCK_EN();
		}

	} else {

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
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

			// 1. configure the RTSR

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {

			// 1. configure both FTSR and FTSR

		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR

		// 3. enable the EXTI interrupt delivery using IMR

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
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function changes configuration of IRQ
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * @param[in]         - enable or disable interrupt
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDi) {

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

}
