#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#define RS 0x20																		/* PB5 mask for reg select */
#define EN 0x80     															/* PB7 mask for enable */

/* delay function */
void delayMs(int n);

/* interrupt handler functions */
void EXTI9_5_IRQHandler(void);
void EXTI3_IRQHandler(void);
	
/* LCD functions */
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);

/* global variables for cruise control */
static int cruise_control_flag = 0;
static double desired_frequency = 0.0;

/* global variables for PID control */
static double previous_error;
static double current_error;
static double integration_sum;
static double KP;
static double KI;
static int KD;
static double modification;

/* global variables for frequency calculation and output */
static double frequency = 0.0;
static char frequency_str[23];
static unsigned int last = 0;
static unsigned int current;
static unsigned int period;
static unsigned int result;

int main(void){
		LCD_init();																		/* initialize LCD */
	
		// configure PA6 as input of TIM3 CH1
		RCC->AHB1ENR |=  1;             							/* enable GPIOA clock */
    GPIOA->MODER &= ~(uint32_t)0x00003000;    		/* clear pin mode */
    GPIOA->MODER |=  0x00002000;    							/* set pin to alternate function */
    GPIOA->AFR[0] &= ~(uint32_t)0x0F000000;   		/* clear pin AF bits */
    GPIOA->AFR[0] |= 0x02000000;    							/* set pin to AF2 for TIM3 CH1 */
		
		// configure TIM3 to do input capture with prescaler ...
    RCC->APB1ENR |= 2;              							/* enable TIM3 clock */
    TIM3->PSC = 16000 - 1;          							/* divided by 16000 */
    TIM3->CCMR1 = 0x41;             							/* set CH1 to capture at every edge */
    TIM3->CCER = 0x01;              							/* enable CH 1 capture falling edges */
    TIM3->CR1 = 1;                  							/* enable TIM3 */
		
		// configure PA5 as output of TIM2 CH1
		GPIOA->AFR[0] |= 0x00100000;    							/* PA5 pin for tim2 */
    GPIOA->MODER &= ~(uint32_t)0x00000C00;
    GPIOA->MODER |=  0x00000800;
	
		// configure TIM2 to be used in PWM mode
		RCC->APB1ENR |= 1;              							/* enable TIM2 clock */
    TIM2->PSC = 10 - 1;             							/* divided by 10 */
    TIM2->ARR = 26667 - 1;          							/* divided by 26667 */
    TIM2->CNT = 0;
    TIM2->CCMR1 = 0x0060;           							/* PWM mode */
    TIM2->CCER = 1;                 							/* enable PWM Ch1 */
    TIM2->CR1 = 1;                  							/* enable timer */
		
		// configure PA1 as input of ADC CH1
    RCC->AHB1ENR |=  1;	            							/* enable GPIOA clock */
    GPIOA->MODER |=  0xC;           							/* PA1 analog */
		GPIOA->MODER |= 0x00004000;
		
    // configure ADC1, it gets voltage from the potentiometer
    RCC->APB2ENR |= 0x00000100;     							/* enable ADC1 clock */
    ADC1->CR2 = 0;                  							/* SW trigger */
    ADC1->SQR3 = 1;                 							/* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 							/* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 							/* enable ADC1 */

		// configure PC9 as an external triggered interrupt source
		RCC->APB2ENR |= 0x00004000;             			/* enable SYSCFG clock */
		SYSCFG->EXTICR[2] &= ~(uint32_t)0x00F0;   		/* clear port selection for EXTI9 */
		SYSCFG->EXTICR[2] |= 0x0020;        					/* select port C for EXTI9 */
    EXTI->IMR |= 0x200;                						/* unmask EXTI9 */
		EXTI->FTSR |= 0x200;             			  			/* select falling edge trigger */
		GPIOC->MODER &= ~(uint32_t)0x000A0000;				/* set PC9 as input mode*/
		GPIOC->PUPDR = 0x40000;
		NVIC_EnableIRQ(EXTI9_5_IRQn);     						/* enable interrupt in NVIC */
		
		// configure PC3 as an external triggered interrupt source
		SYSCFG->EXTICR[0] |= 0x2000;      						/* select port c for EXTI 3 */
		EXTI->IMR |= 0x8;
		EXTI->FTSR |= 0x8;
		GPIOC->MODER &= ~(uint32_t)0x00A0;						/* set PC3 as input mode */
		GPIOC->PUPDR |= 0x0040;
		NVIC_EnableIRQ(EXTI3_IRQn);										/* enable interrupt in NVIC */

	
    while(1){                          
			// In the beginning of every loop, check cruise control flag to determine which sub while loop to continue
			
			/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
				while (!cruise_control_flag){  
						// SUB WHILE LOOP 1: not in cruise control mode
						ADC1->CR2 |= 0x40000000;    					/* start a conversion */
						while(!(ADC1->SR & 2)) {}       			/* wait for conv complete */
						result = ADC1->DR;              			/* read conversion result */
						result = (unsigned int)6.512 * result;/* 6.512 is a calculated (26666/4095 = 6.512) value to map ADC output(max 4095) to
																										 TIM2->CCR1(max 26666) */
						TIM2->CCR1 = result;
						while (!(TIM3->SR & 2)){}							/* wait until input edge is captured */
								
						current = TIM3->CCR1;       					/* read captured counter value */
						period = current - last;    					/* calculate the period */
						last = current;
						frequency = 1000.0 / period;
						last = current;
						LCD_command(1);
						sprintf(frequency_str, "The F is: %f. \r\n", frequency);
						for (unsigned int i = 0; i <= strlen(frequency_str); i++){
								LCD_data(frequency_str[i]);
						}
				}

				/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
				while (cruise_control_flag){   
						// SUB WHILE 2: in cruise control mode
						/* PID control */
						while (!(TIM3->SR & 2)){}
						current = TIM3->CCR1;       					/* read captured counter value */
						period = current - last;    					/* calculate the period */
						last = current;
						frequency = 1000.0 / period;
						last = current;
						LCD_command(1);
						sprintf(frequency_str, "Current: %f. \r\n", frequency);
						for (unsigned int i = 0; i <= strlen(frequency_str); i++){
								LCD_data(frequency_str[i]);
						}
						current_error = desired_frequency - frequency;
						integration_sum += (current_error * period);
						
						// apply modification term to the current output to achieve closed-loop control
						modification = KP * current_error + KI * integration_sum + KD * (current_error-previous_error) / period;
						TIM2->CCR1 += (unsigned int)modification;
						previous_error = current_error;
				}
		}
}

void delayMs(int n) {
		/* 16 MHz SYSCLK */
    int i;
    /* Configure SysTick */
    SysTick->LOAD = 16000;  											/* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       											/* clear current value register */
    SysTick->CTRL = 0x5;    											/* Enable the timer */
    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0){} 	/* wait until the COUNTFLAG is set */
    }
    SysTick->CTRL = 0;      											/* Stop the timer (Enable = 0) */
}

void LCD_init(void) {
    PORTS_init();

    delayMs(20);                									/* LCD controller reset sequence */
    LCD_nibble_write(0x30, 0);
    delayMs(5);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);

    LCD_nibble_write(0x20, 0);  									/* use 4-bit data mode */
    delayMs(1);
    LCD_command(0x28);          									/* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          									/* move cursor right */
    LCD_command(0x01);          									/* clear screen, move cursor to home */
    LCD_command(0x0F);          									/* turn on display, cursor blinking */
}

void PORTS_init(void) {
    RCC->AHB1ENR |=  0x07;          							/* enable GPIOA/B/C clock */

    // PORTB 5 for LCD R/S, PORTB 7 for LCD EN
    GPIOB->MODER &= ~(uint32_t)0x0000CC00;    		/* clear pin mode */
		GPIOB->MODER |=  0x00004500;    							/* set pin output mode */
    GPIOB->BSRR = 0x00800000;       							/* turn off EN */
		GPIOA->MODER =  0x00000000;
		GPIOA->PUPDR = 0x00015400;
    // PC4-PC7 for LCD D4-D7, respectively. */
    GPIOC->MODER &= ~(uint32_t)0x0000FF00;    		/* clear pin mode */
    GPIOC->MODER |=  0x00005500;    							/* set pin output mode */
}

void LCD_nibble_write(char data, unsigned char control) {
    // populate data bits 
    GPIOC->BSRR = 0x00F00000;       							/* clear data bits */
    GPIOC->BSRR = data & 0xF0;      							/* set data bits */
    // set R/S bit 
    if (control & RS)
        GPIOB->BSRR = RS;
    else
        GPIOB->BSRR = RS << 16;
    // pulse E
    GPIOB->BSRR = EN;
    delayMs(0);
    GPIOB->BSRR = EN << 16;
}

void LCD_command(unsigned char command) {
    LCD_nibble_write(command & 0xF0, 0);    			/* upper nibble first */
    LCD_nibble_write((char)(command << 4), 0);    /* then lower nibble */
    if (command < 4)
        delayMs(2);         											/* command 1 and 2 needs up to 1.64ms */
    else
        delayMs(1);         											/* all others 40 us */
}

void LCD_data(char data) {

    LCD_nibble_write(data & 0xF0, RS);      			/* upper nibble first */
    LCD_nibble_write((char)(data << 4), RS);      /* then lower nibble */

    delayMs(1);
}
void EXTI9_5_IRQHandler(void) {
		// if cruise control interrupt triggered
		cruise_control_flag = 1;											/* change the flag meaning going to the other sub while loop */
		desired_frequency = frequency;								/* record the frequence when entering cruise control used as a reference */
		previous_error = 0;
		current_error = 0;
	  integration_sum = 0;
		KP = 4500;
		KI = 0.0001;
		KD = 4500;

		LCD_command(1);
		LCD_data('C');
		LCD_data('r');
		LCD_data('u');
		LCD_data('i');
		LCD_data('s');
		LCD_data('e');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data('C');
		LCD_data('o');
		LCD_data('n');
		LCD_data('t');
		LCD_data('r');
		LCD_data('o');
		LCD_data('l');
		delayMs(2000);
		LCD_command(1);

		EXTI->PR |= 0x200;														/* clear interrupt flag */
} 

void EXTI3_IRQHandler(void){
		// if regular mode interrupt triggered
		cruise_control_flag = 0;											/* change the flag to its default value */
		LCD_command(1);
		LCD_data('R');
		LCD_data('e');
		LCD_data('g');
		LCD_data('u');
		LCD_data('l');
		LCD_data('a');
		LCD_data('r');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data(' ');
		LCD_data('M');
		LCD_data('o');
		LCD_data('d');
		LCD_data('e');
		delayMs(2000);
		LCD_command(1);
		EXTI->PR |= 0x8;															/* clear interrupt flag */
}
