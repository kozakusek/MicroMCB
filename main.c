#include <stm32.h>

#include "accelerometer.h"

int main() {
    accelerometer_init();
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
    __WFI();
}
