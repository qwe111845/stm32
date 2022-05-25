/*
 * stm32f429i.h
 *
 *  Created on: May 16, 2022
 *      Author: lin
 */

#ifndef INC_STM32F429I_H_
#define INC_STM32F429I_H_
#include<stdint.h>
#define __vo volatile
/*
 *  base address of Flash and SRAM memoris
 */

#define FLASH_BASEADDR                  0x08000000U // FLASH 記憶體位址
#define SRAM1_BASEADDR                  0x20000000U // 112KB SRAM1 位址
#define SRAM2_BASEADDR                  0x2001C000U // 16KB  SRAM1 位址
#define SRAM3_BASEADDR                  0x20020000U // 64KB  SRAM1 位址
#define ROM_BASEADDR                    0x1FFF0000U // System memory 就是 ROM 位址
#define SRAM                            SRAM1_BASEADDR

/*
 *  AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE                     0x40000000U
#define APB1PERIPAH_BASE                PERIPH_BASE
#define APB2PERIPAH_BASE                0x40010000U
#define AHB1PERIPAH_BASE                0x40020000U
#define AHB2PERIPAH_BASE                0x50000000U

/*
 *  Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                  (AHB1PERIPAH_BASE + 0x0000)
#define GPIOB_BASEADDR                  (AHB1PERIPAH_BASE + 0x0400)
#define GPIOC_BASEADDR                  (AHB1PERIPAH_BASE + 0x0800)
#define GPIOD_BASEADDR                  (AHB1PERIPAH_BASE + 0x0C00)
#define GPIOE_BASEADDR                  (AHB1PERIPAH_BASE + 0x1000)
#define GPIOF_BASEADDR                  (AHB1PERIPAH_BASE + 0x1400)
#define GPIOG_BASEADDR                  (AHB1PERIPAH_BASE + 0x1800)
#define GPIOH_BASEADDR                  (AHB1PERIPAH_BASE + 0x1C00)
#define GPIOI_BASEADDR                  (AHB1PERIPAH_BASE + 0x2000)
#define GPIOJ_BASEADDR                  (AHB1PERIPAH_BASE + 0x2400)
#define GPIOK_BASEADDR                  (AHB1PERIPAH_BASE + 0x2800)
#define RCC_BASEADDR                    (AHB1PERIPAH_BASE + 0x3800)
/*
 *  Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR                   (APB1PERIPAH_BASE + 0x5400)
#define I2C2_BASEADDR                   (APB1PERIPAH_BASE + 0x5800)
#define I2C3_BASEADDR                   (APB1PERIPAH_BASE + 0x5C00)

#define SPI2_BASEADDR                   (APB1PERIPAH_BASE + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPAH_BASE + 0x3C00)

#define USART2_BASEADDR                 (APB1PERIPAH_BASE + 0x4400)
#define USART3_BASEADDR                 (APB1PERIPAH_BASE + 0x4800)

#define UART4_BASEADDR                  (APB1PERIPAH_BASE + 0x4C00)
#define UART5_BASEADDR                  (APB1PERIPAH_BASE + 0x5000)

/*
 *  Base addresses of peripherals which are hanging on APB2 bus
 *  TODO: Complete for all other peripherals
 */

#define SPI1_BASEADDR                   (APB2PERIPAH_BASE + 0x3000)

#define USART1_BASEADDR                 (APB2PERIPAH_BASE + 0x1000)
#define USART6_BASEADDR                 (APB2PERIPAH_BASE + 0x1400)

#define EXTI_BASEADDR                   (APB2PERIPAH_BASE + 0x3C00)

#define SYSCFG_BASEADDR                 (APB2PERIPAH_BASE + 0x3800)


/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4X family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct {
	__vo uint32_t MODER;             // GPIO port mode register,                         Address offset: 0x00
	__vo uint32_t OTYPER;            // GPIO port output type register,                  Address offset: 0x04
	__vo uint32_t OSPEEDR;           // GPIO port output speed register,                 Address offset: 0x08
	__vo uint32_t PUPDR;             // GPIO port pull-up/pull-down register,            Address offset: 0x0C
	__vo uint32_t IDR;               // GPIO port input data register,                   Address offset: 0x10
	__vo uint32_t ODR;               // GPIO port output data register,                  Address offset: 0x14
	__vo uint32_t BSRR;              // GPIO port bit set/reset register,                Address offset: 0x18
	__vo uint32_t LCKR;              // GPIO port configuration lock register,           Address offset: 0x1C
	__vo uint32_t AFR[2];            /* GPIO alternate function low register AFR[0],     Address offset: 0x20
	                                    GPIO alternate function high register AFR[1],    Address offset: 0x24 */
} GPIO_RegDef_t;


typedef struct {
	__vo uint32_t CR;             // RCC clock control register,                                  Address offset: 0x00
	__vo uint32_t PLLCFGR;        // RCC PLL configuration register,                              Address offset: 0x04
	__vo uint32_t CFGR;           // RCC clock configuration register,                            Address offset: 0x08
	__vo uint32_t CIR;            // RCC clock interrupt register,                                Address offset: 0x0C
	__vo uint32_t AHB1RSTR;       // RCC AHB1 peripheral reset register,                          Address offset: 0x10
	__vo uint32_t AHB2RSTR;       // RCC AHB2 peripheral reset register,                          Address offset: 0x14
	__vo uint32_t AHB3RSTR;       // RCC AHB3 peripheral reset register,                          Address offset: 0x18
	uint32_t      RESERVED0;      // Reserved, 0x1c
	__vo uint32_t APB1RSTR;       // RCC APB1 peripheral reset register,                          Address offset: 0x20
	__vo uint32_t APB2RSTR;       // RCC APB2 peripheral reset register,                          Address offset: 0x24
	uint32_t      RESERVED1[2];   // Reserved, 0x28-2C
    __vo uint32_t AHB1ENR;        // RCC AHB1 peripheral clock enable register,                   Address offset: 0x30
	__vo uint32_t AHB2ENR;        // RCC AHB2 peripheral clock enable register,                   Address offset: 0x34
	__vo uint32_t AHB3ENR;        // RCC AHB3 peripheral clock enable register,                   Address offset: 0x38
	uint32_t      RESERVED2;      // Reserved, 0x3C
	__vo uint32_t APB1ENR;        // RCC APB1 peripheral clock enable register,                   Address offset: 0x40
	__vo uint32_t APB2ENR;        // RCC APB2 peripheral clock enable register,                   Address offset: 0x44
	uint32_t      RESERVED3[2];   // Reserved, 0x48-4C
    __vo uint32_t AHB1LPENR;      // RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
	__vo uint32_t AHB2LPENR;      // RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
	__vo uint32_t AHB3LPENR;      // RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
	uint32_t      RESERVED4;      // Reserved, 0x5C
	__vo uint32_t APB1LPENR;      // RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
	__vo uint32_t APB2LPENR;      // RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
	uint32_t      RESERVED5[2];   // Reserved, 0x68-6C
	__vo uint32_t BDCR;           // RCC Backup domain control register,                          Address offset: 0x70
	__vo uint32_t CSR;            // RCC clock control & status register,                         Address offset: 0x74
	uint32_t      RESERVED6[2];   // Reserved, 0x78-7C
    __vo uint32_t SSCGR;          // RCC clock control & status register,                         Address offset: 0x80
	__vo uint32_t PLLI2SCFGR;     // RCC PLLI2S configuration register,                           Address offset: 0x84
	__vo uint32_t PLLSAICFGR;     // RCC PLL configuration register,                              Address offset: 0x88
	__vo uint32_t DCKCFGR;        // RCC Dedicated Clock Configuration Register ,                 Address offset: 0x8C

} RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */
typedef struct {
	__vo uint32_t IMR;           // RCC Backup domain control register,                          Address offset: 0x70
	__vo uint32_t EMR;            // RCC clock control & status register,                         Address offset: 0x74
    __vo uint32_t RTSR;          // RCC clock control & status register,                         Address offset: 0x80
	__vo uint32_t FTSR;     // RCC PLLI2S configuration register,                           Address offset: 0x84
	__vo uint32_t SWIER;     // RCC PLL configuration register,                              Address offset: 0x88
	__vo uint32_t PR;        // RCC Dedicated Clock Configuration Register ,                 Address offset: 0x8C
} EXTI_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA                           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                           ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                           ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                           ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ                           ((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK                           ((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC                             ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXIT                            ((RCC_RegDef_t*) EXTI_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLOCK_EN()       (RCC->AHB1ENR |= (1<<10))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1<<15))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1<<5))


/*
 * Clock Enable Macros for STSCFG peripherals
 */

#define STSCFG_PCLK_EN()          (RCC->APB2ENR |= (1<<14))



/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLOCK_DIS()       (RCC->AHB1ENR &= ~(1<<10))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DIS()          (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DIS()        (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DIS()        (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DIS()        (RCC->APB1ENR &= ~(1<<18))
#define USART6_PCLK_DIS()        (RCC->APB2ENR &= ~(1<<5))


/*
 * Clock Disable Macros for STSCFG peripherals
 */

#define STSCFG_PCLK_DIS()        (RCC->APB2ENR &= ~(1<<14))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(2)
#define GPIOC_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(4)
#define GPIOD_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(5)
#define GPIOE_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(7)
#define GPIOF_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(8)
#define GPIOG_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)
#define GPIOJ_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9));} while(0)
#define GPIOK_REG_RESET()        do {(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10));} while(0)

// some generic marcos

#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET


#include "stm32f429zi_gpio_driver.h"



#endif /* INC_STM32F429I_H_ */
