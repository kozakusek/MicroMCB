#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <stm32.h>
#include <gpio.h>

#include "usart.h"
#include "i2c_conf.h"
#include "accelerometer.h"

#define PCLK1_HZ 16000000U

#define CTRL_REG1 0x20
#define CTRL_REG3 0x22

#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D

#define SCL_GPIO GPIOB
#define SDA_GPIO GPIOB
#define SCL_PIN 8
#define SDA_PIN 9

#define LIS35DE_ADDR 0x1C

#define CTRL_REG1_PD 0b01000000
#define CTRL_REG1_XEN 0b00000001
#define CTRL_REG1_YEN 0b00000010
#define CTRL_REG1_ZEN 0b00000100

#define CTRL_REG3_PP 0b00000000
#define CTRL_REG3_IH 0b00000000
#define CTRL_REG3_I1DR 0b00000100

#define INT1_GPIO GPIOA
#define INT1_PIN 1
#define INT1_MASK (1 << INT1_PIN)

#define MODE_BTN_GPIO GPIOA
#define MODE_BTN_PIN 0
#define USER_BTN_GPIO GPIOC
#define USER_BTN_PIN 13

typedef int8_t reg_t;

typedef int8_t reg_val_t;

typedef void (*write_callback_t)();

typedef void (*read_callback_t)(reg_val_t);

typedef enum {
    IDLE,
    WRITE_SB,
    WRITE_ADDR,
    WRITE_TXE,
    WRITE_BTF,
    READ_SB_1,
    READ_ADDR_1,
    READ_BTF,
    READ_SB_2,
    READ_ADDR_2,
    READ_RXNE,
} state_enum;

#define WRITE WRITE_SB
#define READ READ_SB_1

typedef struct  {
    state_enum state;
    reg_t reg;
    reg_val_t val;
    union { 
        write_callback_t write_callback;
        read_callback_t read_callback;
    } callback ;

} i2c_state_t;

i2c_state_t i2c_state = { .state = IDLE };

void i2c_write(reg_t reg, reg_val_t val, write_callback_t callback) {
    i2c_state.state = WRITE;
    i2c_state.reg = reg;
    i2c_state.val = val;
    i2c_state.callback.write_callback = callback;

    I2C1->CR2 |= I2C_CR2_ITBUFEN;
    I2C1->CR1 |= I2C_CR1_START;
}

void i2c_read(reg_t reg, read_callback_t callback) {
    i2c_state.state = READ;
    i2c_state.reg = reg;
    i2c_state.callback.read_callback = callback;

    I2C1->CR2 |= I2C_CR2_ITBUFEN;
    I2C1->CR1 |= I2C_CR1_START;
}

/******************************************************************************/

void init_buttons() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOCEN;

    GPIOinConfigure(MODE_BTN_GPIO, 
                    MODE_BTN_PIN, 
                    GPIO_PuPd_DOWN,
                    EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);
    EXTI->PR = EXTI_PR_PR0;
    NVIC_EnableIRQ(EXTI0_IRQn);

    GPIOinConfigure(USER_BTN_GPIO, 
                    USER_BTN_PIN, 
                    GPIO_PuPd_UP,
                    EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);
    EXTI->PR = EXTI_PR_PR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

bool MODE_STATE = false;

void EXTI0_IRQHandler() {
    EXTI->PR = EXTI_PR_PR0;
    MODE_STATE = (MODE_BTN_GPIO->IDR >> MODE_BTN_PIN) & 1;
}

void user_btn_action();

void EXTI15_10_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13;
        if ((USER_BTN_GPIO->IDR >> USER_BTN_PIN) & 1) {
            user_btn_action();
        }
    }
}

/******************************************************************************/
void config_cr3() {
    GPIOinConfigure(INT1_GPIO, 
                    INT1_PIN, 
                    GPIO_PuPd_DOWN, 
                    EXTI_Mode_Interrupt, 
                    EXTI_Trigger_Rising);
    NVIC_EnableIRQ(EXTI1_IRQn);
    EXTI->PR = EXTI_PR_PR1;
    i2c_write(CTRL_REG3, 
              CTRL_REG3_PP  | 
              CTRL_REG3_IH  | 
              CTRL_REG3_I1DR,
              NULL);
}

void config_cr1() {
    i2c_write(CTRL_REG1, 
              CTRL_REG1_PD  | 
              CTRL_REG1_XEN | 
              CTRL_REG1_YEN | 
              CTRL_REG1_ZEN,
              &config_cr3);
}

void clear_cr3() {
    i2c_write(CTRL_REG3, 0, &config_cr1);
}

void clear_cr1() {
    i2c_write(CTRL_REG1, 0, &clear_cr3);
}

void accelerometer_init() {
    usart_init();
    init_buttons();
    i2c_configure(PCLK1_HZ);
    GPIOafConfigure(SCL_GPIO, 
                    SCL_PIN, 
                    GPIO_OType_OD, 
                    GPIO_Low_Speed, 
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);
    GPIOafConfigure(SDA_GPIO, 
                    SDA_PIN, 
                    GPIO_OType_OD, 
                    GPIO_Low_Speed, 
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);
    clear_cr1();
}

/******************************************************************************/

#define SENSITIVITY_THRESHOLD 10
#define MAX_XY_VALUE 56
#define MAX_ACCEL_VALUE 127
#define START_BYTE 0x01

reg_val_t X;
reg_val_t Y;

reg_val_t XD;
reg_val_t YD;

void user_btn_action() {
    XD = X;
    YD = Y;
}

reg_val_t inner_clip(reg_val_t val, reg_val_t threshold) {
    if (-threshold < val && val < threshold) {
        return 0;
    } 
    return val;   
}

reg_val_t outer_clip(reg_val_t val, reg_val_t threshold) {
    if (val < -threshold) {
        return -threshold;
    } else if (threshold < val) {
        return threshold;
    }
    return val;
}

reg_val_t rescale(reg_val_t val, reg_val_t old_max, reg_val_t new_max) {
    return (val * new_max) / old_max;
}

reg_val_t clip_and_rescale(reg_val_t val) {
    val = inner_clip(val, SENSITIVITY_THRESHOLD);
    val = outer_clip(val, MAX_XY_VALUE);
    val = rescale(val, MAX_XY_VALUE, MAX_ACCEL_VALUE);
    return val;
}

#define K 10

reg_val_t avg_step_x(reg_val_t val) {
    static reg_val_t avg_buff[K] = {0};
    static reg_val_t avg = 0;
    static size_t avg_i = 0;

    avg -= avg_buff[avg_i] / K;
    avg_buff[avg_i] = val;
    avg += val / K;
    avg_i = (avg_i + 1) % K;

    return avg;
}

reg_val_t avg_step_y(reg_val_t val) {
    static reg_val_t avg_buff[K] = {0};
    static reg_val_t avg = 0;
    static size_t avg_i = 0;

    avg -= avg_buff[avg_i] / K;
    avg_buff[avg_i] = val;
    avg += val / K;
    avg_i = (avg_i + 1) % K;

    return avg;
}

reg_val_t safe_sub(reg_val_t val, reg_val_t sub) {
    int tmp = val - sub;
    if (tmp < INT8_MIN) {
        return INT8_MIN;
    } else if (INT8_MAX < tmp) {
        return INT8_MAX;
    }
    return tmp;
}

void data_ready();

void finalise_reads(reg_val_t _) {
    char buf[3] = {
        START_BYTE, 
        safe_sub((X / 2) * 2, XD),
        safe_sub((Y / 2) * 2, YD)
    };
    usart_write(&buf[0]);
    usart_write(&buf[1]);
    usart_write(&buf[2]);
    
    if ((INT1_GPIO->IDR & INT1_MASK) && !(EXTI->PR & EXTI_PR_PR1)) {
        data_ready();
    }
}

void read_z(reg_val_t val) {
    if (!MODE_STATE)
        Y = avg_step_y(clip_and_rescale(val));
    i2c_read(OUT_Z, &finalise_reads);
}

void read_y(reg_val_t val) {
    if (!MODE_STATE)
        X = avg_step_x(clip_and_rescale(val));
    i2c_read(OUT_Y, &read_z);
}

void read_x(reg_val_t _) {
    i2c_read(OUT_X, &read_y);
}

void data_ready() {
    read_x(0);
}

void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        data_ready();
    }
}

void I2C1_READ_IRQHandler() {
    if (i2c_state.state == READ_SB_1 && (I2C1->SR1 & I2C_SR1_SB)) {
        I2C1->DR = LIS35DE_ADDR << 1;
        i2c_state.state = READ_ADDR_1;
    } else if (i2c_state.state == READ_ADDR_1 && (I2C1->SR1 & I2C_SR1_ADDR)) { 
        I2C1->SR2;
        I2C1->DR = i2c_state.reg;
        i2c_state.state = READ_BTF;
    } else if (i2c_state.state == READ_BTF && (I2C1->SR1 & I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_START;
        i2c_state.state = READ_SB_2;
    } else if (i2c_state.state == READ_SB_2 && (I2C1->SR1 & I2C_SR1_SB)) {
        I2C1->DR = (LIS35DE_ADDR << 1) | 1;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c_state.state = READ_ADDR_2;
    } else if (i2c_state.state == READ_ADDR_2 && (I2C1->SR1 & I2C_SR1_ADDR)) {
        I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        i2c_state.state = READ_RXNE;
    } else if (i2c_state.state == READ_RXNE && (I2C1->SR1 & I2C_SR1_RXNE)) {
        I2C1->CR2 ^= I2C_CR2_ITBUFEN;
        i2c_state.state = IDLE;
        if (i2c_state.callback.read_callback != NULL) {
            i2c_state.callback.read_callback(I2C1->DR);
        }
    } else {
        I2C1->SR1;
    }
}

void I2C1_WRITE_IRQHandler() {
    if (i2c_state.state == WRITE_SB && (I2C1->SR1 & I2C_SR1_SB)) {
        I2C1->DR = LIS35DE_ADDR << 1;
        i2c_state.state = WRITE_ADDR;
    } else if (i2c_state.state == WRITE_ADDR && (I2C1->SR1 & I2C_SR1_ADDR)) {
        I2C1->SR2;
        I2C1->DR = i2c_state.reg;
        i2c_state.state = WRITE_TXE;
    } else if (i2c_state.state == WRITE_TXE && (I2C1->SR1 & I2C_SR1_TXE)) {
        I2C1->DR = i2c_state.val;
        i2c_state.state = WRITE_BTF;
    } else if (i2c_state.state == WRITE_BTF && (I2C1->SR1 & I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->CR2 ^= I2C_CR2_ITBUFEN;
        i2c_state.state = IDLE;
        if (i2c_state.callback.write_callback != NULL) {
            i2c_state.callback.write_callback();
        }
    } else {
        I2C1->SR1;
    }
}

void I2C1_EV_IRQHandler() {
    if (i2c_state.state >= READ) {
        I2C1_READ_IRQHandler();
    } else if (i2c_state.state >= WRITE) {
        I2C1_WRITE_IRQHandler();
    } else {
        I2C1->SR1;
    }
}