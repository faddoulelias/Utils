/*
 * main.c
 *
 *  Created on: 27 mars 2023
 *      Author: ad
 */

#include "stm32f0xx.h"
#include "stm.h"
#include "delay.h"

#define PERIOD 10

// variable globale
// declarer la variable globale timebase_irq avec le qualificatif volatile
volatile uint8_t timebase_irq = 0;
uint8_t output = 0;
unsigned int i = 0;

int main()
{
  initTimer(TIMER_14, PERIOD);
  initGPIO(GPIO_A);
  initGPIO(GPIO_C);
  initADC1(GPIO_C, 0);
  initDAC1(4);
  initConsole();

  setPinMode(GPIO_A, 5, GPIO_MODE_OUTPUT);
  setPinMode(GPIO_C, 13, GPIO_MODE_INPUT);

  setTimerEventPriority(TIMER_14, 0);

  while (1)
  {
    if (timebase_irq)
    {
      timebase_irq = 0;
    }
  }
}

void TIM14_IRQHandler(void)
{
  lowerTimerFlag(TIMER_14);
  setPinState(GPIO_A, 15, HIGH);
  i++;

  serialPrint("Valeur ADC = %d\r\n", ADC1->DR);

  // Suiveur
  DAC->DHR12R1 = ADC1->DR;

  timebase_irq = 1;
}
