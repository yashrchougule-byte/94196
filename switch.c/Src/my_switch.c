/*
 * my_switch.c
 *
 *  Created on: Dec 17, 2025
 *      Author: kiran_z6dopa8
 */

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"
#include"my_switch.h"


void switch_init(void)
{
	RCC->AHB1ENR |= BV(0);
	GPIOA->MODER &= ~(BV(0) | BV(1) );



	GPIOA->OSPEEDR &= ~(BV(0) | BV(1));

	GPIOA->PUPDR &= ~(BV(0) | BV(1));

}
int switch_status(void)
{
	if ((GPIOA->IDR) & BV(0) )
	{
		return 1 ; // switch is press
	}
	else
	{
		return 0 ; // switch is nor press
	}
}
