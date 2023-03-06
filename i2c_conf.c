// Modified /arm/stm32/src/i2c_configure.c

#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <xcat.h>

#include "i2c_conf.h"

#define I2C_GPIO_N  B
#define I2C_SCL_PIN 8
#define I2C_SDA_PIN 9

#define I2C_GPIO                xcat(GPIO, I2C_GPIO_N)
#define RCC_AHB1ENR_I2C_GPIO_EN xcat3(RCC_AHB1ENR_GPIO, I2C_GPIO_N, EN)

#define I2C_SCL_MASK (1 << I2C_SCL_PIN)
#define I2C_SDA_MASK (1 << I2C_SDA_PIN)

#define I2C_SCL_HIGH() I2C_GPIO->BSRR = I2C_SCL_MASK
#define I2C_SCL_LOW()  I2C_GPIO->BSRR = I2C_SCL_MASK << 16
#define I2C_SDA_HIGH() I2C_GPIO->BSRR = I2C_SDA_MASK
#define I2C_SDA_LOW()  I2C_GPIO->BSRR = I2C_SDA_MASK << 16

#define I2C_IS_SCL_LOW() ((I2C_GPIO->IDR & I2C_SCL_MASK) == 0)
#define I2C_IS_SDA_LOW() ((I2C_GPIO->IDR & I2C_SDA_MASK) == 0)

/* Częstotliwość taktowania szyny I2C w Hz */
#define I2C_SPEED_HZ 100000U

/* Parametr pclk to częstotliwość taktowania szyny PCLK1 w Hz. */
void i2c_configure(unsigned pclk) {
  RCC->AHB1ENR |= RCC_AHB1ENR_I2C_GPIO_EN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  __NOP();

  /* Najpierw odwieś szynę I2C. To jest czysta heureza. */

  I2C_SCL_HIGH();
  I2C_SDA_HIGH(); // NACK
  GPIOoutConfigure(I2C_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);
  GPIOoutConfigure(I2C_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  // Taktuj szynę z częstotliwością około 12,5 kHz.
  unsigned half_cycle = pclk / 100000;

  // Dokończ ewentualnie niedokończoną transmisję, wystawiając NACK.
  Delay(half_cycle);
  for (unsigned i = 0; i < 9 || I2C_IS_SDA_LOW(); ++i) {
    I2C_SCL_LOW();
    Delay(half_cycle);
    I2C_SCL_HIGH();
    Delay(half_cycle);
  }

  Delay(half_cycle);
  I2C_SDA_LOW();
  Delay(half_cycle);
  I2C_SCL_LOW(); // START
  Delay(2 * half_cycle);
  I2C_SCL_HIGH(); // STOP
  Delay(half_cycle);
  I2C_SDA_HIGH();
  Delay(half_cycle);

  /* Konfiguruj linię SCL na wyprowadzeniu PB8, a linię SDA na wyprowadzeniu
  PB9, funkcja alternatywna 4. Są zewnętrzne rezystory podciągające. */
  GPIOafConfigure(I2C_GPIO, I2C_SCL_PIN, GPIO_OType_OD, GPIO_Low_Speed,
                  GPIO_PuPd_NOPULL, GPIO_AF_I2C1);
  GPIOafConfigure(I2C_GPIO, I2C_SDA_PIN, GPIO_OType_OD, GPIO_Low_Speed,
                  GPIO_PuPd_NOPULL, GPIO_AF_I2C1);

  /* Konfiguruj szynę w trybie standardowym. */
  I2C1->CR1 = 0;

  /* Konfiguruj częstotliwość taktowania szyny.
  Źródła przerwań aktywujemy dynamicznie. */
  I2C1->CCR = pclk / (I2C_SPEED_HZ << 1);
  pclk /= 1000000;
  I2C1->CR2 = pclk | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;
  I2C1->TRISE = pclk + 1;

  /* Włącz interfejs. */
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  I2C1->CR1 |= I2C_CR1_PE;
}
