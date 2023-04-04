#ifndef STM_H
#define STM_H
#include "stm32f0xx.h"

#define SET_BIT(REG, BIT, VALUE) REG = (REG & ~(1 << BIT)) | (VALUE << BIT)
#define SET_PAIR(REG, BIT, VALUE) REG = (REG & ~(3 << BIT)) | (VALUE << BIT)
#define TOGGLE_BIT(REG, BIT) REG ^= (1 << BIT)

typedef enum GPIO_
{
    GPIO_A,
    GPIO_B,
    GPIO_C,
    GPIO_D,
    GPIO_E,
    GPIO_F
} GPIO;

typedef enum GPIO_MODE_
{
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTERNATE = 2,
    GPIO_MODE_ANALOG = 3
} GPIO_MODE;

typedef enum PIN_STATE_
{
    LOW = 0,
    HIGH = 1
} PIN_STATE;

typedef enum TIMER_ID_
{
    TIMER_2 = 0,
    TIMER_3 = 1,
    TIMER_6 = 2,
    TIMER_7 = 3,
    TIMER_14 = 4
} TIMER_ID;

typedef enum PULL_MODE_
{
    NO_PULL_UP_PULL_DOWN = 0,
    PULL_UP = 1,
    PULL_DOWN = 2,
    RESERVED = 3
} PULL_MODE;

/**
 * @brief Initialize a timer
 *
 * @param gpio
 */
void initGPIO(GPIO gpio);

/**
 * @brief Set the pull mode of a pin
 *
 * @param gpio GPIO port
 * @param port GPIO pin
 * @param mode GPIO pull mode
 */
void setPinMode(GPIO gpio, int port, GPIO_MODE mode);

/**
 * @brief Set the state of a pin
 *
 * @param gpio GPIO port
 * @param port  GPIO pin
 * @param state  GPIO state
 */
void setPinState(GPIO gpio, int port, PIN_STATE state);

/**
 * @brief Toggle the state of a pin
 *
 * @param gpio GPIO port
 * @param port  GPIO pin
 */
void togglePinState(GPIO gpio, int port);

/**
 * @brief Set the pull mode of a pin
 *
 * @param gpio GPIO port
 * @param port GPIO pin
 * @param mode PULL_MODE
 */
void setPinPullMode(GPIO gpio, int port, PULL_MODE mode);

/**
 * @brief Read the state of a pin
 *
 * @param gpio GPIO port
 * @param port GPIO pin
 * @return PIN_STATE : LOW or HIGH
 */
PIN_STATE readPinState(GPIO gpio, int port);

/**
    Initialize the Analog to Digital Converter (ADC1) on the specified GPIO pin

    @param gpio GPIO port
    @param port GPIO pin

    @warning Only support PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5
*/
void initADC1(GPIO gpio, int port);

/**
    Initialize the Digital to Analog Converter (DAC1) on the specified GPIO pin

    @param port GPIO pin

    @warning Only support PA4
*/
void initDAC1(int port);

/**
 * @brief Initilize a timer
 *
 * @param timer_id
 * @param period (in ms)
 */
void initTimer(TIMER_ID timer_id, uint16_t period);

void lowerTimerFlag();

/**
 * @brief Start a timer and set its interrupt priority
 *
 * @param timer_id
 * @param priority
 */
void setTimerEventPriority(TIMER_ID timer_id, uint8_t priority);

/**
 * @brief Initialize the console
 *
 */
void initConsole();

int serialPrint(const char *format, ...);
int stringPrint(char *out, const char *format, ...);

#endif // STM_H
