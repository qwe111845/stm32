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

/*********************** START: Processor Specific Details *********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0                      ((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1                      ((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2                      ((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3                      ((__vo uint32_t*) 0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ICER0                      ((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1                      ((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2                      ((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3                      ((__vo uint32_t*) 0xE000E18C)


/*
 * ARM Cortex Mx Processor NVIC Priority register Addresses
 */
#define NVIC_PR_BASE_ADDR               ((__vo uint32_t*) 0xE000E400)


/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority register Addresses
 */
#define NO_PR_BITS_IMPLEMENTED          4

/*
 *  base address of Flash and SRAM memories
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
 */

#define SPI1_BASEADDR                   (APB2PERIPAH_BASE + 0x3000)
#define SPI4_BASEADDR                   (APB2PERIPAH_BASE + 0x3400)
#define SPI5_BASEADDR                   (APB2PERIPAH_BASE + 0x5000)
#define SPI6_BASEADDR                   (APB2PERIPAH_BASE + 0x5400)

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
	__vo uint32_t CR1;               // SPI control register 1,                          Address offset: 0x00
	__vo uint32_t CR2;               // SPI control register 2,                          Address offset: 0x04
	__vo uint32_t SR;                // SPI status register,                             Address offset: 0x08
	__vo uint32_t DR;                // SPI data register,                               Address offset: 0x0C
	__vo uint32_t CRCPR;             // SPI CRC polynomial register,                     Address offset: 0x10
	__vo uint32_t RXCRCR;            // SPI RX CRC register,                             Address offset: 0x14
	__vo uint32_t TXCRCR;            // SPI TX CRC register,                             Address offset: 0x18
	__vo uint32_t I2SCFGR;           // SPI_I2S configuration register,                  Address offset: 0x1C
	__vo uint32_t I2SPR;             // SPI_I2S prescaler register,                      Address offset: 0x20
} SPI_RegDef_t;

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
	__vo uint32_t IMR;      // Interrupt mask register,                     Address offset: 0x00
	__vo uint32_t EMR;      // Event mask register,                         Address offset: 0x04
    __vo uint32_t RTSR;     // Rising trigger selection register,           Address offset: 0x08
	__vo uint32_t FTSR;     // Falling trigger selection register,          Address offset: 0x0C
	__vo uint32_t SWIER;    // Software interrupt event register,           Address offset: 0x10
	__vo uint32_t PR;       // Pending register,                            Address offset: 0x14
} EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;      // SYSCFG memory re-map register,                          Address offset: 0x00
	__vo uint32_t PMC;         // SYSCFG peripheral mode configuration register,          Address offset: 0x04
    __vo uint32_t EXTICR[4];   // SYSCFG external interrupt configuration register 1-4,   Address offset: 0x08-0x14
	uint32_t      RESERVED[2]; // Reserved, 0x18-1C
    __vo uint32_t CMPCR;       //Compensation cell control register,                      Address offset: 0x20
	uint32_t      RESERVED2[2]; // Reserved, 0x24-28

} SYSCFG_RegDef_t;

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

#define EXTI                            ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG                          ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1                            ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2                            ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3                            ((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4                            ((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5                            ((SPI_RegDef_t*) SPI5_BASEADDR)
#define SPI6                            ((SPI_RegDef_t*) SPI6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()       (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()       (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()       (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()       (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()       (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()       (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()       (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()       (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()       (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()       (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()       (RCC->AHB1ENR |= (1<<10))


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
#define SPI4_PCLK_EN()          (RCC->APB1ENR |= (1<<13))
#define SPI5_PCLK_EN()          (RCC->APB1ENR |= (1<<20))
#define SPI6_PCLK_EN()          (RCC->APB1ENR |= (1<<21))

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

#define SYSCFG_PCLK_EN()          (RCC->APB2ENR |= (1<<14))



/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DIS()       (RCC->AHB1ENR &= ~(1<<10))


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
#define SPI4_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<13))
#define SPI5_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<20))
#define SPI6_PCLK_DIS()          (RCC->APB1ENR &= ~(1<<21))
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


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()        do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB1RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()        do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()        do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()        do {(RCC->APB2RSTR |= (1 << 13)); (RCC->APB1RSTR &= ~(1 << 13));} while(0)
#define SPI5_REG_RESET()        do {(RCC->APB2RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));} while(0)
#define SPI6_REG_RESET()        do {(RCC->APB2RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)

/*
 * returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)( (x== GPIOA) ? 0: \
		                          (x== GPIOB) ? 1: \
		                          (x== GPIOC) ? 2: \
		                          (x== GPIOD) ? 3: \
				                  (x== GPIOE) ? 4: \
						          (x== GPIOF) ? 5: \
						          (x== GPIOG) ? 6: \
						          (x== GPIOH) ? 7: \
						          (x== GPIOI) ? 8: \
						          (x== GPIOJ) ? 9: \
						          (x== GPIOK) ? 10: 0 )


/*
 * IRQ(Interrupt Request) Number of STM32F429x MCU
 * Note: update these macros with valid values according to your MCU
 */
#define IRQ_NO_EXTI0         6
#define IRQ_NO_EXTI1         7
#define IRQ_NO_EXTI2         8
#define IRQ_NO_EXTI3         9
#define IRQ_NO_EXTI4         10
#define IRQ_NO_EXTI9_5       23
#define IRQ_NO_EXTI15_10     40


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0        0
#define NVIC_IRQ_PRI1        1
#define NVIC_IRQ_PRI2        2
#define NVIC_IRQ_PRI3        3
#define NVIC_IRQ_PRI4        4
#define NVIC_IRQ_PRI5        5
#define NVIC_IRQ_PRI6        6
#define NVIC_IRQ_PRI7        7
#define NVIC_IRQ_PRI8        8
#define NVIC_IRQ_PRI9        9
#define NVIC_IRQ_PRI10       10
#define NVIC_IRQ_PRI11       11
#define NVIC_IRQ_PRI12       12
#define NVIC_IRQ_PRI13       13
#define NVIC_IRQ_PRI14       14
#define NVIC_IRQ_PRI15       15


// some generic marcos

#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET
#define FLAG_SET         SET
#define FLAG_RESET       RESET

/********************************************************************************************
 *  Bit position definition of SPI peripheral
 ********************************************************************************************/

#define SPI_CR1_CPHA     0
#define SPI_CR1_CPOL     1
#define SPI_CR1_MSTR     2
#define SPI_CR1_BR       3
#define SPI_CR1_SPE      6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSI      8
#define SPI_CR1_SSM      9
#define SPI_CR1_RXONLY   10
#define SPI_CR1_DFF      11
#define SPI_CR1_CRCNEXT  12
#define SPI_CR1_CRCEN    13
#define SPI_CR1_BIDIOE   14
#define SPI_CR1_BIDIMODE 15

#define SPI_CR2_RXDMAEN  0
#define SPI_CR2_TXDMAEN  1
#define SPI_CR2_SSOE     2
#define SPI_CR2_FRF      4
#define SPI_CR2_ERRIE    5
#define SPI_CR2_RXNEIE   6
#define SPI_CR2_TXNEIE   7

#define SPI_SR_RXNE      0
#define SPI_SR_TXE       1
#define SPI_SR_CHSIDE    2
#define SPI_SR_UDR       3
#define SPI_SR_CRCERR    4
#define SPI_SR_MODF      5
#define SPI_SR_OVR       6
#define SPI_SR_BSY       7
#define SPI_SR_FRE       8



#include "stm32f429zi_gpio_driver.h"
#include "stm32f429zi_spi_driver.h"



#endif /* INC_STM32F429I_H_ */
