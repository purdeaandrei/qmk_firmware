/* Copyright 2021 Andrei Purdea (portions)
 * Copyright (C) 2019 Elia Ritterbusch (portions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This library follows the convention of the AVR i2c_master library.
 * As a result addresses are expected to be already shifted (addr << 1).
 */

#include "i2c_master.h"
#include "timer.h"
#include "gpio.h"
#include "wait.h"
#include <stdbool.h>

#ifndef I2C_BITBANG_SDA_PIN
#    error "You must set I2C_BITBANG_SDA_PIN in config.h, in order to user the i2c bitbang driver"
/* Define it to something, to avoid follow-up error messages from the compiler */
#    define I2C_BITBANG_SDA_PIN 0
#endif

#ifndef I2C_BITBANG_SCL_PIN
#    error "You must set I2C_BITBANG_SCL_PIN in config.h, in order to user the i2c bitbang driver"
/* Define it to something, to avoid follow-up error messages from the compiler */
#    define I2C_BITBANG_SCL_PIN 0
#endif

#ifndef I2C_BITBANG_ENABLE_INTERNAL_PULLUPS
#    ifdef PAL_MODE_OUTPUT_OPENDRAIN
#        define INITIALISE_PIN(pin) palSetLineMode((pin), PAL_MODE_OUTPUT_OPENDRAIN)
#        define WRITE_PIN(pin, value) writePin((pin), (value))
#        define READ_PIN(pin) readPin(pin)
#    else
#        define INITIALISE_PIN(pin) setPinInput(pin)
#        define WRITE_PIN(pin, value) do { if (value) { setPinInput(pin); } else { writePin((pin), 0); setPinOutput(pin); } } while (0)
#        define READ_PIN(pin) readPin(pin)
#    endif
#else
#    if defined(PAL_STM32_MODE_OUTPUT) && defined(PAL_STM32_OTYPE_OPENDRAIN) && defined(PAL_STM32_PUPDR_PULLUP)
#        define INITIALISE_PIN(pin) palSetLineMode((pin), PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP)
#        define WRITE_PIN(pin, value) writePin((pin), (value))
#        define READ_PIN(pin) readPin(pin)
#    else
#        ifdef __AVR__
#            error "It's not possible to instantly switch between driving a strong 0, and input with weak pull-up enabled on AVR."
#        else
#            error "If i2c bitbang with internal pull-ups enabled is possible on this MCU, it has not yet been implemented."
#        endif
#    endif
#endif

#define I2C_BITBANG_RECOVERY_CLOCKS 9

#ifndef I2C_BITBANG_FREQUENCY_KHZ
#    if defined(I2C_BITBANG_STANDARD_MODE)
#        define I2C_BITBANG_FREQUENCY_KHZ 100
#    else
#        define I2C_BITBANG_FREQUENCY_KHZ 400
#    endif
#endif

#if (I2C_BITBANG_FREQUENCY_KHZ <= 100) && !defined(I2C_BITBANG_STANDARD_MODE)
#    define I2C_BITBANG_STANDARD_MODE
#endif

#define I2C_BITBANG_CLOCK_PERIOD_NS ((1000000 + I2C_BITBANG_FREQUENCY_KHZ - 1) / (I2C_BITBANG_FREQUENCY_KHZ))

#ifdef I2C_BITBANG_STANDARD_MODE
#    define T_HOLD_TIME_START_NS 4000
#    define T_SETUP_TIME_START_NS 4700
#    define T_SCL_CLOCK_LOW_PERIOD_NS 4700
#    define T_SCL_CLOCK_HIGH_PERIOD_NS 4000
#    define T_ACK_VALID_TIME_NS 3450
#    define T_DATA_VALID_TIME_NS 3450
#    define T_SETUP_TIME_STOP_NS 4000
#    define T_BUS_FREE_TIME 4700
#else
/* Fast mode: */
#    define T_HOLD_TIME_START_NS 600
#    define T_SETUP_TIME_START_NS 600
#    define T_SCL_CLOCK_LOW_PERIOD_NS 1300
#    define T_SCL_CLOCK_HIGH_PERIOD_NS 600
#    define T_ACK_VALID_TIME_NS 900
#    define T_DATA_VALID_TIME_NS 900
#    define T_SETUP_TIME_STOP_NS 600
#    define T_BUS_FREE_TIME 1300
#endif

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS (MAX(I2C_BITBANG_CLOCK_PERIOD_NS - T_SCL_CLOCK_LOW_PERIOD_NS, T_SCL_CLOCK_HIGH_PERIOD_NS))

#ifndef I2C_BITBANG_WAIT_NS
#    ifdef PROTOCOL_CHIBIOS
#        if (PORT_SUPPORTS_RT == FALSE) || !defined(STM32_SYSCLK)
#            warning "chSysPolledDelayX method not supported on this platform, using inaccurate delay method. Bitbang i2c will work but will be much slower."
#            define I2C_BITBANG_WAIT_NS(time) wait_us(((time) + 999) / 1000)
#        else
#            define I2C_BITBANG_WAIT_NS(time) chSysPolledDelayX(((STM32_SYSCLK + 999999) / 1000000 * (time) + 999) / 1000)
#            define TIMEOUT_START_TIME_TYPE rtcnt_t
#            define TIMEOUT_GET_COUNTER chSysGetRealtimeCounterX
#            define TIMEOUT_HAS_IT_TIMED_OUT(initcounter, timeout_ms) (((rtcnt_t)(chSysGetRealtimeCounterX() - (initcounter))) >= MS2RTC(STM32_SYSCLK, timeout_ms))
#            define USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
#            define SCL_PERIOD_IN_SYSCLK_CYCLES ((STM32_SYSCLK + (I2C_BITBANG_FREQUENCY_KHZ * 1000) - 1) / (I2C_BITBANG_FREQUENCY_KHZ * 1000))
#        endif
#    else
#        ifdef __AVR__
#            ifndef F_CPU
#                define F_CPU 16000000
#            endif
#            define I2C_BITBANG_WAIT_NS(time) __builtin_avr_delay_cycles(((time) * 1000ULL + (1000000000000ULL / F_CPU) - 1) / (1000000000000ULL / F_CPU))
#        else
#            warning "Using inaccurate delay method. Bitbang i2c will work but will be much slower."
#            define I2C_BITBANG_WAIT_NS(time) wait_us(((time) + 999) / 1000)
#        endif
#    endif
#endif

#ifndef TIMEOUT_START_TIME_TYPE
#    define TIMEOUT_START_TIME_TYPE uint16_t
#    define TIMEOUT_GET_COUNTER timer_read
#    define TIMEOUT_HAS_IT_TIMED_OUT(initcounter, timeout_ms) ( ((uint16_t)(timer_read() - initcounter)) >= timeout_ms)
#endif

#define I2C_BITBANG_HARDCODED_TIMEOUT_ON_STOP_MS 20

bool started = false;
bool has_been_used = false;

static i2c_status_t i2c_try_to_recover_potentially_locked_up_bus(uint16_t timeout) {
    TIMEOUT_START_TIME_TYPE init_timeout_counter = TIMEOUT_GET_COUNTER();
    for (int i=0; i<I2C_BITBANG_RECOVERY_CLOCKS; i++) {
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
        while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
            /* wait for clock stretching */
            if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
                return I2C_STATUS_TIMEOUT;
            }
        }
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
        if (READ_PIN(I2C_BITBANG_SDA_PIN)) {
            break;
        }
    }
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 0); /* preparing for stop bit */
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
    while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
        /* wait for clock stretching */
        if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
            return I2C_STATUS_TIMEOUT;
        }
    }
    I2C_BITBANG_WAIT_NS(T_SETUP_TIME_STOP_NS);
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1); /* stop condition final */
    I2C_BITBANG_WAIT_NS(T_BUS_FREE_TIME);
    started = false;
    return I2C_STATUS_SUCCESS;
}

void i2c_init(void) {
    INITIALISE_PIN(I2C_BITBANG_SDA_PIN);
    INITIALISE_PIN(I2C_BITBANG_SCL_PIN);
    started = false;
    has_been_used = false;
}

i2c_status_t i2c_start(uint8_t address, uint16_t timeout) {
    if (!has_been_used) {
        has_been_used = true;
        i2c_status_t ret = i2c_try_to_recover_potentially_locked_up_bus(address);
        if (I2C_STATUS_SUCCESS != ret) {
            return ret;
        }
    }
    TIMEOUT_START_TIME_TYPE init_timeout_counter = TIMEOUT_GET_COUNTER();
    if (!started) {
        if ((READ_PIN(I2C_BITBANG_SCL_PIN) == 0) ||
            (READ_PIN(I2C_BITBANG_SDA_PIN) == 0)) {
            return I2C_STATUS_ERROR;
        }
    } else {
        /* Restart condition */
        I2C_BITBANG_WAIT_NS(MAX(T_ACK_VALID_TIME_NS, T_SCL_CLOCK_LOW_PERIOD_NS));
        if ((READ_PIN(I2C_BITBANG_SCL_PIN) != 0) ||
            (READ_PIN(I2C_BITBANG_SDA_PIN) == 0)) {
            return I2C_STATUS_ERROR;
        }
        /* SDA pin was expected to have been released before, so it should be high */
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
        while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
            /* wait for clock stretching */
            if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
                return I2C_STATUS_TIMEOUT;
            }
        }
        I2C_BITBANG_WAIT_NS(T_SETUP_TIME_START_NS);
    }
    /* start condition */
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 0);
    I2C_BITBANG_WAIT_NS(T_HOLD_TIME_START_NS);
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    TIMEOUT_START_TIME_TYPE init_cycle_counter = TIMEOUT_GET_COUNTER();
#endif
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    for (int8_t i=1; i<=8; i++) {
        WRITE_PIN(I2C_BITBANG_SDA_PIN, ((address >> (8-i)) & 1));
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
        while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
            /* wait for clock stretching */
            if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
                return I2C_STATUS_TIMEOUT;
            }
        }
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
        while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 3)) {
            /* note: -3 is not architecture-specific, it comes from a fact that there's at least 3 distinct operations
             * that are not counted: substraction with underflow, comparison, and reading of a new counter value
             */
            /* wait for SCL cycle length to fill up */
        }
        init_cycle_counter = TIMEOUT_GET_COUNTER();
#else
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    }
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1); /* release the SDA pin to get ACK/NACK */
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
    while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
        /* wait for clock stretching */
        if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
            return I2C_STATUS_TIMEOUT;
        }
    }
    uint8_t ack = READ_PIN(I2C_BITBANG_SDA_PIN);

#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
    while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 2)) {
        /* note: -2 is not architecture-specific, it comes from a fact that there's at least 2 distinct operations
         * that are not counted: substraction with underflow, and comparison
         */
        /* wait for SCL cycle length to fill up */
    }
#else
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    started = true;
    if (ack != 0) return I2C_STATUS_ERROR;
    return I2C_STATUS_SUCCESS;
}

i2c_status_t i2c_write(uint8_t data, uint16_t timeout) {
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    TIMEOUT_START_TIME_TYPE init_cycle_counter = TIMEOUT_GET_COUNTER();
#endif
    TIMEOUT_START_TIME_TYPE init_timeout_counter = TIMEOUT_GET_COUNTER();
    for (int8_t i=7; i>=0; i--) {
        WRITE_PIN(I2C_BITBANG_SDA_PIN, ((data >> i) & 1));
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
        while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
            /* wait for clock stretching */
            if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
                return I2C_STATUS_TIMEOUT;
            }
        }
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
        while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 3)) {
            /* note: -3 is not architecture-specific, it comes from a fact that there's at least 3 distinct operations
             * that are not counted: substraction with underflow, comparison, and reading of a new counter value
             */
            /* wait for SCL cycle length to fill up */
        }
        init_cycle_counter = TIMEOUT_GET_COUNTER();
#else
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    }
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1); /* release the SDA pin to get ACK/NACK */
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
    while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
        /* wait for clock stretching */
        if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
            return I2C_STATUS_TIMEOUT;
        }
    }
    uint8_t ack = READ_PIN(I2C_BITBANG_SDA_PIN);
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
    while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 2)) {
        /* note: -2 is not architecture-specific, it comes from a fact that there's at least 2 distinct operations
         * that are not counted: substraction with underflow, and comparison
         */
        /* wait for SCL cycle length to fill up */
    }
#else
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    if (ack != 0) return I2C_STATUS_ERROR;
    return I2C_STATUS_SUCCESS;
}

static int16_t i2c_read_acknack(uint16_t timeout, uint8_t ack) {
    uint8_t data = 0;
    TIMEOUT_START_TIME_TYPE init_timeout_counter = TIMEOUT_GET_COUNTER();
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1); /* release the SDA pin so that the device can talk back */
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    TIMEOUT_START_TIME_TYPE init_cycle_counter = TIMEOUT_GET_COUNTER();
    /* Note: there is no write of the SCL pin after reading the counter here, but with
     * all other delays involving calling this function, and writing the SDA pin above,
     * the clock period constraint is not violated.
     */
#endif
    for (int8_t i=7; i>=0; i--) {
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
        while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
            /* wait for clock stretching */
            if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
                return I2C_STATUS_TIMEOUT;
            }
        }
        data = (data << 1) | !!READ_PIN(I2C_BITBANG_SDA_PIN);
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
        while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 3)) {
            /* note: -3 is not architecture-specific, it comes from a fact that there's at least 3 distinct operations
             * that are not counted: substraction with underflow, comparison, and reading of a new counter value
             */
            /* wait for SCL cycle length to fill up */
        }
        init_cycle_counter = TIMEOUT_GET_COUNTER();
#else
        I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
        WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    }
    I2C_BITBANG_WAIT_NS(T_DATA_VALID_TIME_NS);
    /* Now the device must have released the SDA pin so we can respond with an ack/nack */
    if (READ_PIN(I2C_BITBANG_SDA_PIN) == 0) {
        return I2C_STATUS_ERROR;
    }
    WRITE_PIN(I2C_BITBANG_SDA_PIN, ack); /* set the desired ack value */
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
    while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
        /* wait for clock stretching */
        if ((timeout != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, timeout)) {
            return I2C_STATUS_TIMEOUT;
        }
    }
#ifdef USE_SYSCLOCK_TIMER_FOR_CYCLE_LENGTH
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_HIGH_PERIOD_NS);
    while (((TIMEOUT_START_TIME_TYPE)(TIMEOUT_GET_COUNTER() - init_cycle_counter)) < (SCL_PERIOD_IN_SYSCLK_CYCLES - 2)) {
        /* note: -2 is not architecture-specific, it comes from a fact that there's at least 2 distinct operations
         * that are not counted: substraction with underflow, and comparison
         */
        /* wait for SCL cycle length to fill up */
    }
#else
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_ACTUAL_HIGH_PERIOD_NS);
#endif
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 0);
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1); /* release the SDA pin */
    return data;
}

int16_t i2c_read_ack(uint16_t timeout) {
    return i2c_read_acknack(timeout, 0);
}

int16_t i2c_read_nack(uint16_t timeout) {
    return i2c_read_acknack(timeout, 1);
}

void i2c_stop(void)
{
    TIMEOUT_START_TIME_TYPE init_timeout_counter = TIMEOUT_GET_COUNTER();
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 0);
    I2C_BITBANG_WAIT_NS(T_SCL_CLOCK_LOW_PERIOD_NS);
    WRITE_PIN(I2C_BITBANG_SCL_PIN, 1);
    while (READ_PIN(I2C_BITBANG_SCL_PIN) == 0) {
        /* wait for clock stretching */
        if ((I2C_BITBANG_HARDCODED_TIMEOUT_ON_STOP_MS != I2C_TIMEOUT_INFINITE) && TIMEOUT_HAS_IT_TIMED_OUT(init_timeout_counter, I2C_BITBANG_HARDCODED_TIMEOUT_ON_STOP_MS)) {
            return;
        }
    }
    I2C_BITBANG_WAIT_NS(T_SETUP_TIME_STOP_NS);
    WRITE_PIN(I2C_BITBANG_SDA_PIN, 1);
    I2C_BITBANG_WAIT_NS(T_BUS_FREE_TIME);
    started = false;
}

i2c_status_t i2c_transmit(uint8_t address, const uint8_t* data, uint16_t length, uint16_t timeout) {
    i2c_status_t status = i2c_start(address | I2C_WRITE, timeout);

    for (uint16_t i = 0; i < length && status >= 0; i++) {
        status = i2c_write(data[i], timeout);
    }

    i2c_stop();

    return status;
}

i2c_status_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length, uint16_t timeout) {
    i2c_status_t status = i2c_start(address | I2C_READ, timeout);

    for (uint16_t i = 0; i < (length - 1) && status >= 0; i++) {
        status = i2c_read_ack(timeout);
        if (status >= 0) {
            data[i] = status;
        }
    }

    if (status >= 0) {
        status = i2c_read_nack(timeout);
        if (status >= 0) {
            data[(length - 1)] = status;
        }
    }

    i2c_stop();

    return (status < 0) ? status : I2C_STATUS_SUCCESS;
}

i2c_status_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, const uint8_t* data, uint16_t length, uint16_t timeout) {
    i2c_status_t status = i2c_start(devaddr | 0x00, timeout);
    if (status >= 0) {
        status = i2c_write(regaddr, timeout);

        for (uint16_t i = 0; i < length && status >= 0; i++) {
            status = i2c_write(data[i], timeout);
        }
    }

    i2c_stop();

    return status;
}

i2c_status_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length, uint16_t timeout) {
    i2c_status_t status = i2c_start(devaddr, timeout);
    if (status < 0) {
        goto error;
    }

    status = i2c_write(regaddr, timeout);
    if (status < 0) {
        goto error;
    }

    status = i2c_start(devaddr | 0x01, timeout);

    for (uint16_t i = 0; i < (length - 1) && status >= 0; i++) {
        status = i2c_read_ack(timeout);
        if (status >= 0) {
            data[i] = status;
        }
    }

    if (status >= 0) {
        status = i2c_read_nack(timeout);
        if (status >= 0) {
            data[(length - 1)] = status;
        }
    }

error:
    i2c_stop();

    return (status < 0) ? status : I2C_STATUS_SUCCESS;
}
