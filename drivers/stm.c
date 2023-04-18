#include <stm.h>

void initGPIO(GPIO gpio)
{
    switch (gpio)
    {
    case GPIO_A:
        SET_BIT(RCC->AHBENR, 17, 1);
        break;
    case GPIO_B:
        SET_BIT(RCC->AHBENR, 18, 1);
        break;
    case GPIO_C:
        SET_BIT(RCC->AHBENR, 19, 1);
        break;
    case GPIO_D:
        SET_BIT(RCC->AHBENR, 20, 1);
        break;
    case GPIO_E:
        SET_BIT(RCC->AHBENR, 21, 1);
        break;
    case GPIO_F:
        SET_BIT(RCC->AHBENR, 22, 1);
        break;
    }
}

void setPinMode(GPIO gpio, int port, GPIO_MODE mode)
{
    initGPIO(gpio);
    switch (gpio)
    {
    case GPIO_A:
        SET_PAIR(GPIOA->MODER, port * 2, mode);
        break;
    case GPIO_B:
        SET_PAIR(GPIOB->MODER, port * 2, mode);
        break;
    case GPIO_C:
        SET_PAIR(GPIOC->MODER, port * 2, mode);
        break;
    case GPIO_D:
        SET_PAIR(GPIOD->MODER, port * 2, mode);
        break;
    case GPIO_E:
        SET_PAIR(GPIOE->MODER, port * 2, mode);
        break;
    case GPIO_F:
        SET_PAIR(GPIOF->MODER, port * 2, mode);
        break;
    }
    setPinPullMode(gpio, port, NO_PULL_UP_PULL_DOWN);
}

void setPinState(GPIO gpio, int port, PIN_STATE state)
{
    switch (gpio)
    {
    case GPIO_A:
        SET_BIT(GPIOA->ODR, port, state);
        break;
    case GPIO_B:
        SET_BIT(GPIOB->ODR, port, state);
        break;
    case GPIO_C:
        SET_BIT(GPIOC->ODR, port, state);
        break;
    case GPIO_D:
        SET_BIT(GPIOD->ODR, port, state);
        break;
    case GPIO_E:
        SET_BIT(GPIOE->ODR, port, state);
        break;
    case GPIO_F:
        SET_BIT(GPIOF->ODR, port, state);
        break;
    }
}

void togglePinState(GPIO gpio, int port)
{
    switch (gpio)
    {
    case GPIO_A:
        TOGGLE_BIT(GPIOA->ODR, port);
        break;
    case GPIO_B:
        TOGGLE_BIT(GPIOB->ODR, port);
        break;
    case GPIO_C:
        TOGGLE_BIT(GPIOC->ODR, port);
        break;
    case GPIO_D:
        TOGGLE_BIT(GPIOD->ODR, port);
        break;
    case GPIO_E:
        TOGGLE_BIT(GPIOE->ODR, port);
        break;
    case GPIO_F:
        TOGGLE_BIT(GPIOF->ODR, port);
        break;
    }
}

void setPinPullMode(GPIO gpio, int port, PULL_MODE mode)
{
    initGPIO(gpio);
    switch (gpio)
    {
    case GPIO_A:
        SET_PAIR(GPIOA->PUPDR, port * 2, mode);
        break;
    case GPIO_B:
        SET_PAIR(GPIOB->PUPDR, port * 2, mode);
        break;
    case GPIO_C:
        SET_PAIR(GPIOC->PUPDR, port * 2, mode);
        break;
    case GPIO_D:
        SET_PAIR(GPIOD->PUPDR, port * 2, mode);
        break;
    case GPIO_E:
        SET_PAIR(GPIOE->PUPDR, port * 2, mode);
        break;
    case GPIO_F:
        SET_PAIR(GPIOF->PUPDR, port * 2, mode);
        break;
    }
}

PIN_STATE readPinState(GPIO gpio, int port)
{
    switch (gpio)
    {
    case GPIO_A:
        return (PIN_STATE)((GPIOA->IDR >> port) & 1);
    case GPIO_B:
        return (PIN_STATE)((GPIOB->IDR >> port) & 1);
    case GPIO_C:
        return (PIN_STATE)((GPIOC->IDR >> port) & 1);
    case GPIO_D:
        return (PIN_STATE)((GPIOD->IDR >> port) & 1);
    case GPIO_E:
        return (PIN_STATE)((GPIOE->IDR >> port) & 1);
    case GPIO_F:
        return (PIN_STATE)((GPIOF->IDR >> port) & 1);
    }

    return UNKNOWN;
}

void initADC1(GPIO gpio, int port)
{

    initGPIO(gpio);
    setPinMode(gpio, port, GPIO_MODE_ANALOG);

    RCC->APB2ENR |= (1 << 9);
    ADC1->CR = 0x00000000;
    ADC1->CFGR1 = 0x00000000;
    ADC1->CFGR2 = 0x00000000;
    ADC1->CHSELR = 0x00000000;

    ADC1->CFGR1 |= (1 << 13);
    ADC1->CFGR1 &= ~(0x03 << 4);
    ADC1->CFGR2 |= (0x01 << 31UL);
    ADC1->SMPR = 0x03;

    if (gpio == GPIO_A)
    {
        switch (port)
        {
        case 0:
            ADC1->CHSELR |= (1 << 0);
            break;
        case 1:
            ADC1->CHSELR |= (1 << 1);
            break;
        case 2:
            ADC1->CHSELR |= (1 << 2);
            break;
        case 3:
            ADC1->CHSELR |= (1 << 3);
            break;
        case 4:
            ADC1->CHSELR |= (1 << 4);
            break;
        case 5:
            ADC1->CHSELR |= (1 << 5);
            break;
        case 6:
            ADC1->CHSELR |= (1 << 6);
            break;
        case 7:
            ADC1->CHSELR |= (1 << 7);
            break;
        }
    }
    else if (gpio == GPIO_B)
    {
        switch (port)
        {
        case 0:
            ADC1->CHSELR |= (1 << 8);
            break;
        case 1:
            ADC1->CHSELR |= (1 << 9);
            break;
        }
    }
    else if (gpio == GPIO_C)
    {
        switch (port)
        {
        case 0:
            ADC1->CHSELR |= (1 << 10);
            break;
        case 1:
            ADC1->CHSELR |= (1 << 11);
            break;
        case 2:
            ADC1->CHSELR |= (1 << 12);
            break;
        case 3:
            ADC1->CHSELR |= (1 << 13);
            break;
        case 4:
            ADC1->CHSELR |= (1 << 14);
            break;
        case 5:
            ADC1->CHSELR |= (1 << 15);
            break;
        }
    }

    ADC1->CR |= (1 << 0);
    ADC1->CR |= (1 << 2);
}

void initDAC1(int port)
{
    initGPIO(GPIO_A);
    setPinMode(GPIO_A, port, GPIO_MODE_ANALOG);

    RCC->APB1ENR |= (1 << 29);

    DAC->CR = 0x00000000;

    // DAC->CR |= (1 << 16);
    switch (port)
    {
    case 4:
        DAC->CR |= (1 << 0);
        break;
    case 5:
        DAC->CR |= (1 << 16);
        break;
    }
}

void initTimer(TIMER_ID timer_id, uint16_t period)
{
    switch (timer_id)
    {
    case TIMER_2:
        RCC->APB1ENR |= (1 << 0);
        TIM2->CR1 = 0x0000;
        TIM2->CR2 = 0x0000;

        TIM2->PSC = (uint16_t)48000 - 1;
        TIM2->ARR = (uint16_t)period - 1;
        TIM2->CR1 |= (1 << 7);
        TIM2->CR1 |= (1 << 0);

        TIM2->DIER |= (1 << 0);
        break;
    case TIMER_3:
        RCC->APB1ENR |= (1 << 1);
        TIM3->CR1 = 0x0000;
        TIM3->CR2 = 0x0000;

        TIM3->PSC = (uint16_t)48000 - 1;
        TIM3->ARR = (uint16_t)period - 1;
        TIM3->CR1 |= (1 << 7);
        TIM3->CR1 |= (1 << 0);

        TIM3->DIER |= (1 << 0);
        break;
    case TIMER_6:
        RCC->APB1ENR |= (1 << 4);
        TIM6->CR1 = 0x0000;
        TIM6->CR2 = 0x0000;

        TIM6->PSC = (uint16_t)48000 - 1;
        TIM6->ARR = (uint16_t)period - 1;
        TIM6->CR1 |= (1 << 7);
        TIM6->CR1 |= (1 << 0);

        TIM6->DIER |= (1 << 0);
        break;
    case TIMER_7:
        RCC->APB1ENR |= (1 << 5);
        TIM7->CR1 = 0x0000;
        TIM7->CR2 = 0x0000;

        TIM7->PSC = (uint16_t)48000 - 1;
        TIM7->ARR = (uint16_t)period - 1;
        TIM7->CR1 |= (1 << 7);
        TIM7->CR1 |= (1 << 0);

        TIM7->DIER |= (1 << 0);
        break;
    case TIMER_14:
        RCC->APB1ENR |= (1 << 8);
        TIM14->CR1 = 0x0000;
        TIM14->CR2 = 0x0000;

        TIM14->PSC = (uint16_t)48000 - 1;
        TIM14->ARR = (uint16_t)period - 1;
        TIM14->CR1 |= (1 << 7);
        TIM14->CR1 |= (1 << 0);

        TIM14->DIER |= (1 << 0);
        break;
    default:
        break;
    }
}

void setTimerEventPriority(TIMER_ID timer_id, uint8_t priority)
{
    switch (timer_id)
    {
    case TIMER_2:
        NVIC_SetPriority(TIM2_IRQn, priority);
        NVIC_EnableIRQ(TIM2_IRQn);
        break;
    case TIMER_3:
        NVIC_SetPriority(TIM3_IRQn, priority);
        NVIC_EnableIRQ(TIM3_IRQn);
        break;
    case TIMER_6:
        NVIC_SetPriority(TIM6_DAC_IRQn, priority);
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
        break;
    case TIMER_7:
        NVIC_SetPriority(TIM7_IRQn, priority);
        NVIC_EnableIRQ(TIM7_IRQn);
        break;
    case TIMER_14:
        NVIC_SetPriority(TIM14_IRQn, priority);
        NVIC_EnableIRQ(TIM14_IRQn);
        break;
    default:
        break;
    }
}

void lowerTimerFlag(TIMER_ID timer_id)
{
    switch (timer_id)
    {
    case TIMER_2:
        TIM2->SR &= ~(1 << 0);
        break;
    case TIMER_3:
        TIM3->SR &= ~(1 << 0);
        break;
    case TIMER_6:
        TIM6->SR &= ~(1 << 0);
        break;
    case TIMER_7:
        TIM7->SR &= ~(1 << 0);
        break;
    case TIMER_14:
        TIM14->SR &= ~(1 << 0);
        break;
    default:
        break;
    }
}

int delay(uint32_t time) // time in ms
{
    uint32_t i = 0;
    uint32_t j = 0;
    for (i = 0; i < time; i++)
    {
        for (j = 0; j < 1000; j++)
        {
        }
    }
    return 0;
}

void initConsole()
{
    RCC->AHBENR |= (1 << 17);
    GPIOA->MODER |= (1 << 5);
    GPIOA->MODER &= ~(1 << 4);
    GPIOA->MODER |= (1 << 7);
    GPIOA->MODER &= ~(1 << 6);
    GPIOA->AFR[0] &= ~(1 << 11);
    GPIOA->AFR[0] &= ~(1 << 10);
    GPIOA->AFR[0] &= ~(1 << 9);
    GPIOA->AFR[0] |= (1 << 8);
    GPIOA->AFR[0] &= ~(1 << 15);
    GPIOA->AFR[0] &= ~(1 << 14);
    GPIOA->AFR[0] &= ~(1 << 13);
    GPIOA->AFR[0] |= (1 << 12);
    RCC->APB1ENR |= (1 << 17);
    USART2->CR1 = 0x00000000;
    USART2->CR2 = 0x00000000;
    USART2->CR3 = 0x00000000;
    RCC->CFGR3 &= ~(1 << 17);
    RCC->CFGR3 &= ~(1 << 16);
    USART2->CR1 |= (1 << 15);
    USART2->BRR = 833;
    USART2->CR1 |= (1 << 3);
    USART2->CR1 |= (1 << 2);
    USART2->CR1 |= (1 << 0);
}
