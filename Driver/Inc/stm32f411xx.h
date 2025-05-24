/*
 * stm32f411xx.h
 *
 *  Created on: 06-Jul-2023
 *      Author: admin
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#define FLASH_BASEADDR 0x80000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR   0x1FFF0000U
#define SRAM           SRAM1_BASEADDR

#define APB1PERIPH_BASE 0x40000000U
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U

#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000)

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0X5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0X3C00)

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0X3000)
#define SPI4_BASEADDR (APB2PERIPH_BASE + 0X3400)
#define SPI5_BASEADDR (APB2PERIPH_BASE + 0X5000)
#define SPI6_BASEADDR (APB2PERIPH_BASE + 0X5400)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0X4800)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0X1400)

#define UART4_BASEADDR (APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0X5000)
#define UART7_BASEADDR (APB1PERIPH_BASE + 0X7800)
#define UART8_BASEADDR (APB1PERIPH_BASE + 0X7C00)

#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0X3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0X3800)

#define __vo volatile

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/*
#define
#define
#define
#define
#define
#define
#define
*/


#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI5_9 23
#define IRQ_NO_EXTI10_15 40

#define NVIC_ISER0 ( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1 ( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2 ( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3 ( (__vo uint32_t*) 0xE000E10C )

#define NVIC_ICER0 ( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1 ( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2 ( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3 ( (__vo uint32_t*) 0xE000E18C )

#define NVIC_IPR0 ( (__vo uint32_t*) 0xE000E400 )
#define NVIC_IPR1 ( (__vo uint32_t*) 0xE000E404 )
#define NVIC_IPR2 ( (__vo uint32_t*) 0xE000E408 )
#define NVIC_IPR3 ( (__vo uint32_t*) 0xE000E40C )

#define NVIC_PR_BASE_ADDRESS ( (__vo uint32_t*) 0xE000E400 )
#define NO_OF_PR_BITS_IMPLEMENTED 4

#define NVIC_IRQ_PRIO1 1
#define NVIC_IRQ_PRIO2 2
#define NVIC_IRQ_PRIO3 3
#define NVIC_IRQ_PRIO4 4
#define NVIC_IRQ_PRIO5 5
#define NVIC_IRQ_PRIO6 6
#define NVIC_IRQ_PRIO7 7
#define NVIC_IRQ_PRIO8 8
#define NVIC_IRQ_PRIO9 9
#define NVIC_IRQ_PRIO10 10
#define NVIC_IRQ_PRIO11 11
#define NVIC_IRQ_PRIO12 12
#define NVIC_IRQ_PRIO13 13
#define NVIC_IRQ_PRIO14 14
#define NVIC_IRQ_PRIO15 15

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

} RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

} EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;

} SYSCFG_RegDef_t;

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define GPIOA_PCLK_EN() ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN() ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN() ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN() ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN() ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN() ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN() ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN() ( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN() ( RCC->AHB1ENR |= (1<<8) )

#define GPIOA_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<8) )

#define SYSCFG_PCLK_EN() ( RCC->APB2ENR |= (1<<14) )

#define GPIOA_REG_RESET() do{ 									\
								((RCC->AHB1RSTR) |= (1<<0));	\
								((RCC->AHB1RSTR) &= ~(1<<0));	\
							} while(0)

#define GPIOB_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<1)); ((RCC->AHB1RSTR) &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<2)); ((RCC->AHB1RSTR) &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<3)); ((RCC->AHB1RSTR) &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<4)); ((RCC->AHB1RSTR) &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<5)); ((RCC->AHB1RSTR) &= ~(1<<5)); } while(0)
#define GPIOG_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<6)); ((RCC->AHB1RSTR) &= ~(1<<6)); } while(0)
#define GPIOH_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<7)); ((RCC->AHB1RSTR) &= ~(1<<7)); } while(0)
#define GPIOI_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<8)); ((RCC->AHB1RSTR) &= ~(1<<8)); } while(0)


#define GPIO_BASEADDR_TO_CODE(X) ( 						\
									(X == GPIOA) ? 0 :	\
									(X == GPIOB) ? 1 : 	\
									(X == GPIOC) ? 2 :	\
									(X == GPIOD) ? 3 :	\
									(X == GPIOE) ? 4 :	\
									(X == GPIOF) ? 5 :	\
									(X == GPIOG) ? 6 :	\
									(X == GPIOH) ? 7 :	\
									(X == GPIOI) ? 8 :0 \
								)



#endif /* STM32F411XX_H_ */
