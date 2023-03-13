/*
 * defs.h
 *
 *  Created on: Mar. 12, 2023
 *      Author: erod2
 */

#ifndef DEFS_H_
#define DEFS_H_

#define LCD_RS GPIO_PIN_8
#define LCD_E  GPIO_PIN_15
#define LCD_D4 GPIO_PIN_14
#define LCD_D5 GPIO_PIN_12
#define LCD_D6 GPIO_PIN_3
#define LCD_D7 GPIO_PIN_4

#define CHARS_PER_LINE 17

#define LCD_RS_PORT GPIOA
#define LCD_E_PORT	GPIOB
#define LCD_D4_PORT GPIOB
#define LCD_D5_PORT GPIOB
#define LCD_D6_PORT GPIOB
#define LCD_D7_PORT GPIOB

#define Ra 1000
#define Rb 1000



/* LCD FUNCTIONS */

void delay_us_sys(uint32_t us);
void LCD_Pulse(void);
void LCD_byte(unsigned char x);
void WriteData(unsigned char x);
void WriteCommand(unsigned char x);
void LCDprint(char * , unsigned char , unsigned char );
void LCD_4BIT (void);


/* Capacitance */
float get_capacitance(float period);
float get_suffix(float capacitance);

/*Period Function And ADC*/
float get_period(void);




#endif /* DEFS_H_ */
