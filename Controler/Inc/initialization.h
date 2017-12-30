#ifndef __INITIALIZATION_H__
#define __INITIALIZATION_H__

//extern int8 speed_right,speed_left;

//extern void PIT0_IRQHandler();

extern void LED_init(void);
extern void initialization(void);
extern void set_priority_irq(void);
extern void PIT0_IRQHandler();
extern void portb_handler();
extern void portc_handler();
extern void uart2_handler();
extern void uart4_handler();
extern void PORTA_IRQHandler();
#endif