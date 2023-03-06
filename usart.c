#include <string.h>

#include <stdbool.h>
#include <gpio.h>
#include <stm32.h>

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable USART_CR1_UE
#define USART_WordLength_8b 0x0000
#define USART_Parity_No 0x0000
#define USART_StopBits_1 0x0000

#define USART_GPIO GPIOA
#define USART_TXD_PIN 9
#define USART_RXD_PIN 10

#define PCLK2_HZ 16000000U
#define BAUD_RATE 9600U

#define DMA_CAN_SEND (DMA2_Stream7->CR & DMA_SxCR_EN) == 0 && \
                     (DMA2->HISR & DMA_HISR_TCIF7) == 0
#define DMA_CAN_READ (DMA2_Stream5->CR & DMA_SxCR_EN) == 0 && \
                     (DMA2->HISR & DMA_HISR_TCIF5) == 0

/************************************ INIT ************************************/

void usart_init_clocking() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN |
                    RCC_APB2ENR_SYSCFGEN;
}

void usart_init_gpio() {
    // TXD
    GPIOafConfigure(USART_GPIO,
                    USART_TXD_PIN,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_USART1);
    // RXD
    GPIOafConfigure(USART_GPIO,
                    USART_RXD_PIN,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_UP,
                    GPIO_AF_USART1);
}

void usart_configure() {
    USART1->CR1 = USART_Mode_Rx_Tx    | 
                  USART_WordLength_8b | 
                  USART_Parity_No;
    USART1->CR2 = USART_StopBits_1;
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART1->BRR = (PCLK2_HZ + (BAUD_RATE / 2U)) / BAUD_RATE;
}

void dma_configure() {
    // SENDER
    DMA2_Stream7->CR = 4U << 25       |
                       DMA_SxCR_PL_1  |
                       DMA_SxCR_MINC  |
                       DMA_SxCR_DIR_0 |
                       DMA_SxCR_TCIE;
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

    // RECEIVER
    DMA2_Stream5->CR = 4U << 25       |
                       DMA_SxCR_PL_1  |
                       DMA_SxCR_MINC  |
                       DMA_SxCR_TCIE;
    DMA2_Stream5->PAR = (uint32_t)&USART1->DR;
}

void dma_enable_interrupts() {
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTCIF5;
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

void enable_usart() {
    USART1->CR1 |= USART_Enable;
}

void usart_init() {
    usart_init_clocking();
    usart_init_gpio();
    usart_configure();
    dma_configure();
    dma_enable_interrupts();
    enable_usart();
}

/*********************************** BUFFER ***********************************/

#define N 300
int r = 0, w = 0;
int free_space = N;
const char* buff[N];

void write(const char* c) {
    int new_write = (w + 1) % N;
    if (new_write != r) {
        buff[w] = c;
        w = new_write;
        free_space = free_space - 1;
    }
}

const char* read() {
    const char *c = buff[r];
    r = (r + 1) % N;
    free_space = free_space + 1;
    return c;
}

inline bool is_empty() {
    return free_space == N;
}

/*********************************** DMAOPS ***********************************/

void DMA_send(const char* c) {
    DMA2_Stream7->M0AR = (uint32_t)c;;
    DMA2_Stream7->NDTR = 1;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
}

void DMA_read_one(char* c) {
    DMA2_Stream5->M0AR = (uint32_t)c;
    DMA2_Stream5->NDTR = 1;
    DMA2_Stream5->CR |= DMA_SxCR_EN;
}

void push_dma() {
    if (!is_empty() && DMA_CAN_SEND) {
        DMA_send(read());
    }
}

// Finished sending
void DMA2_Stream7_IRQHandler() {
    uint32_t isr = DMA2->HISR;
    if (isr & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7;
        push_dma();
    }
}

// Finished reading
void DMA2_Stream5_IRQHandler() {
    uint32_t isr = DMA2->HISR;
    if (isr & DMA_HISR_TCIF5)
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;
}

void usart_write(const char* c) {
    write(c);
    push_dma();
}
