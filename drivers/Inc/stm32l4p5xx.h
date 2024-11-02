/*
 * stm32l4p5xx.h
 *
 *  Created on: Sep 25, 2024
 *      Author: Fernando Aguilar
 */

#ifndef INC_STM32L4P5XX_H_
#define INC_STM32L4P5XX_H_

#include <stdint.h>

#define __vo volatile


/*
 * Base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U /* Flash Memory Base address */
#define SRAM1_BASEADDR						0x20000000U /* SRAM Base address */
#define SRAM2_BASEADDR						0x20020000U
#define SRAM3_BASEADDR						0x20030000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral
 */
#define PERIPH_BASE							0x40000000U
#define APB1PERIPH_BASE						PERIPH_BASE
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE						0x40020C00U
#define AHB2PERIPH_BASE						0x48000000U


/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR						(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR						(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR						(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR						(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR						(AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR						(AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR						(AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR						(AHB2PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR						(AHB2PERIPH_BASE + 0x2000)


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define DMAMUX1_BASEADDR					0x40020800U
#define DMA1_BASEADDR						0x40020000U
#define DMA2_BASEADDR						0x40020400U
#define RCC_BASEADDR						0x40021000U

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SYSCFG_BASEADDR						0x40010000U
#define VREFBUF_BASEADDR					0x40010030U
#define COMP_BASEADDR						0x40010200U
#define EXTI_BASEADDR						0x40010400U
#define FIREWALL_BASEADDR					0x40011C00U
#define TIM1_BASEADDR						0x40012C00U
#define SPI1_BASEADDR						0x40013000U
#define TIM8_BASEADDR						0x40013400U
#define USART1_BASEADDR						0x40013800U
#define TIM15_BASEADDR						0x40014000U
#define TIM16_BASEADDR						0x40014400U
#define TIM17_BASEADDR						0x40014800U


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define TIM2_BASEADDR						(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR						(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR						(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR						(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR						(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR						(APB1PERIPH_BASE + 0x1400)
#define RTC_BASEADDR						(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR						(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR						(APB1PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x5C00)
#define CRS_BASEADDR						(APB1PERIPH_BASE + 0x6000)
#define CAN1_BASEADDR						(APB1PERIPH_BASE + 0x6400)
#define PWR_BASEADDR						(APB1PERIPH_BASE + 0x7000)
#define DAC1_BASEADDR						(APB1PERIPH_BASE + 0x7400)
#define OPAMP_BASEADDR						(APB1PERIPH_BASE + 0x7800)
#define LPTIM1_BASEADDR						(APB1PERIPH_BASE + 0x7C00)
#define LPUART1_BASEADDR					(APB1PERIPH_BASE + 0x8000)
#define I2C4_BASEADDR						(APB1PERIPH_BASE + 0x8400)
#define LPTIM2_BASEADDR						(APB1PERIPH_BASE + 0x9400)




/**********************************************Peripheral register definition structures***************************************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 *
 */

typedef struct
{
	__vo uint32_t MODER;  /* GPIO port mode register. 					*/	/* Address offset: 0x00 */
	__vo uint32_t OTYPER; /* GPIO port output type register. 			*/	/* Address offset: 0x04 */
	__vo uint32_t OSPEEDR;/* GPIO port output speed register.			*/	/* Address offset: 0x08 */
	__vo uint32_t PUPDR;  /* GPIO port pull-up/pull-down register. 		*/	/* Address offset: 0x0C */
	__vo uint32_t IDR;	  /* GPIO port input data register. 			*/	/* Address offset: 0x10 */
	__vo uint32_t ODR;	  /* GPIO port output data register.  			*/	/* Address offset: 0x14 */
	__vo uint32_t BSRR;	  /* GPIO port bit set/reset register. 			*/	/* Address offset: 0x18 */
	__vo uint32_t LCKR;	  /* GPIO port configuration lock register.		*/	/* Address offset: 0x1C */
	__vo uint32_t AFR[2]; /* GPIO alternate function low/high register.	*/ 	/* Address offset: 0x20, 0x24 */
	__vo uint32_t BRR;    /* GPIO port bit reset register.				*/	/* Address offset: 0x28 */
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR;    		/* Clock control register											*/	/* Address offset: 0x00 */
	__vo uint32_t ICSCR;		/* Internal clock sources calibration register 						*/	/* Address offset: 0x04 */
	__vo uint32_t CFGR;			/* Clock configuration register										*/	/* Address offset: 0x08 */
	__vo uint32_t PLLCFGR;		/* PLL configuration register										*/	/* Address offset: 0x0C */
	__vo uint32_t PLLSAI1CFGR;	/* PLLSAI1 configuration register									*/	/* Address offset: 0x10 */
	__vo uint32_t PLLSAI2CFGR;	/* PLLSAI2 configuration register									*/	/* Address offset: 0x14 */
	__vo uint32_t CIER;			/* Clock interrupt enable register									*/	/* Address offset: 0x18 */
	__vo uint32_t CIFR;			/* Clock interrupt flag register									*/	/* Address offset: 0x1C */
	__vo uint32_t CICR;			/* Clock interrupt clear register									*/	/* Address offset: 0x20 */
	uint32_t RESERVED0;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x24 */
	__vo uint32_t AHB1RSTR;		/* AHB1 peripheral reset register									*/	/* Address offset: 0x28 */
	__vo uint32_t AHB2RSTR;		/* AHB2 peripheral reset register									*/	/* Address offset: 0x2C */
	__vo uint32_t AHB3RSTR;		/* AHB3 peripheral reset register									*/	/* Address offset: 0x30 */
	uint32_t RESERVED1;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x34 */
	__vo uint32_t APB1RSTR1;	/* APB1 peripheral reset register 1									*/	/* Address offset: 0x38 */
	__vo uint32_t APB1RSTR2;	/* APB1 peripheral reset register 2									*/	/* Address offset: 0x3C */
	__vo uint32_t APB2RSTR;		/* APB2 peripheral reset register									*/	/* Address offset: 0x40 */
	uint32_t RESERVED2;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x44 */
	__vo uint32_t AHB1ENR;		/* AHB1 peripheral clock enable register							*/	/* Address offset: 0x48 */
	__vo uint32_t AHB2ENR;		/* AHB2 peripheral clock enable register							*/	/* Address offset: 0x4C */
	__vo uint32_t AHB3ENR;		/* AHB3 peripheral clock enable register							*/	/* Address offset: 0x50 */
	uint32_t RESERVED3;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x54 */
	__vo uint32_t APB1ENR1;		/* APB1 peripheral clock enable register 1							*/	/* Address offset: 0x58 */
	__vo uint32_t APB1ENR2;		/* APB1 peripheral clock enable register 2							*/	/* Address offset: 0x5C */
	__vo uint32_t APB2ENR;		/* APB2 peripheral clock enable register							*/	/* Address offset: 0x60 */
	uint32_t RESERVED4;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x64 */
	__vo uint32_t AHB1SMENR;	/* AHB1 peripheral clocks enable in Sleep and Stop modes register	*/	/* Address offset: 0x68 */
	__vo uint32_t AHB2SMENR;	/* AHB2 peripheral clocks enable in Sleep and Stop modes register	*/	/* Address offset: 0x6C */
	__vo uint32_t AHB3SMENR;	/* AHB3 peripheral clocks enable in Sleep and Stop modes register	*/	/* Address offset: 0x70 */
	uint32_t RESERVED5;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x74 */
	__vo uint32_t APB1SMENR1;	/* APB1 peripheral clocks enable in Sleep and Stop modes register 1	*/	/* Address offset: 0x78 */
	__vo uint32_t APB1SMENR2;	/* APB1 peripheral clocks enable in Sleep and Stop modes register 2	*/	/* Address offset: 0x7C */
	__vo uint32_t APB2SMENR;	/* APB2 peripheral clocks enable in Sleep and Stop modes register	*/	/* Address offset: 0x80 */
	uint32_t RESERVED6;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x84 */
	__vo uint32_t CCIPR;		/* Peripherals independent clock configuration register				*/	/* Address offset: 0x88 */
	uint32_t RESERVED7;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0x8C */
	__vo uint32_t BDCR;			/* Backup domain control register									*/	/* Address offset: 0x90 */
	__vo uint32_t CSR;			/* Control/status register											*/	/* Address offset: 0x94 */
	__vo uint32_t CRRCR;		/* Clock recovery RC register										*/	/* Address offset: 0x98 */
	__vo uint32_t CCIPR2;		/* Peripherals independent clock configuration register				*/	/* Address offset: 0x9C */
	uint32_t RESERVED8;			/* Reserved, must be kept at reset value							*/	/* Address offset: 0xA0 */
	__vo uint32_t DLYCFGR;		/* OCTOSPI delay configuration register								*/	/* Address offset: 0xA4 */

}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB2ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB2ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB2ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB2ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB2ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB2ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB2ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB2ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB2ENR |= (1 << 8) )

/*
 * Clock Enable Macros for I2Cx peripherals.
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 23) )
#define I2C4_PCLK_EN()		( RCC->APB1ENR2 |= (1 << 2 ) )

/*
 * Clock Enable Macros for SPIx peripherals.
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR  |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 15) )


/*
 * Clock Enable Macros for USARTx peripherals.
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR  |= (1 << 14) )
#define USART2_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 20) )


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR  |= (1 << 0) )


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		( RCC->AHB2ENR &= ~(1 << 8) )

/*
 * Clock Disable Macros for I2Cx peripherals.
 */

#define I2C1_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 23) )
#define I2C4_PCLK_DI()		( RCC->APB1ENR2 |= ~(1 << 2 ) )

/*
 * Clock Disable Macros for SPIx peripherals.
 */

#define SPI1_PCLK_DI()		( RCC->APB2ENR  |= ~(1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 15) )


/*
 * Clock Disable Macros for USARTx peripherals.
 */

#define USART1_PCLK_DI()	( RCC->APB2ENR  |= ~(1 << 14) )
#define USART2_PCLK_DI()	( RCC->APB1ENR1 |= ~(1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR1 |= ~(1 << 18) )
#define UART4_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 19) )
#define UART5_PCLK_DI()		( RCC->APB1ENR1 |= ~(1 << 20) )


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR  |= ~(1 << 0) )


/*
 * Useful Macros
 */
#define ENABLE				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET



#endif /* INC_STM32L4P5XX_H_ */
