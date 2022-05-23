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

		if (pGPIOx =  GPIOA) {
            GPIOA_PCLK_EN();
		} else if (pGPIOx =  GPIOB) {
            GPIOB_PCLK_EN();
		} else if (pGPIOx =  GPIOC) {
            GPIOC_PCLK_EN();
		} else if (pGPIOx =  GPIOD) {
            GPIOD_PCLK_EN();
		} else if (pGPIOx =  GPIOE) {
            GPIOE_PCLK_EN();
		} else if (pGPIOx =  GPIOF) {
            GPIOF_PCLK_EN();
		} else if (pGPIOx =  GPIOG) {
            GPIOG_PCLK_EN();
		} else if (pGPIOx =  GPIOH) {
            GPIOH_PCLK_EN();
		} else if (pGPIOx =  GPIOI) {
            GPIOI_PCLK_EN();
		} else if (pGPIOx =  GPIOJ) {
            GPIOJ_PCLK_EN();
		} else if (pGPIOx =  GPIOK) {
            GPIOK_PCLK_EN();
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
		pGPIOHandle->pGPIO->MODER |= temp;

	} else {

		// the interrupt mode

	}

	temp = 0;


	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->OSPEEDR |= temp;

	temp = 0;


	// 3. configure the pull-up or pull-down setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->PUPDR |= temp;

	temp = 0;

	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER |= temp;

	temp = 0;

	// 5. configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure the alternate function registers
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
