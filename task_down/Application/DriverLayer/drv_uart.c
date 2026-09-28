/* drv_uart.c - USART5 遥控接收 */

#include "drv_uart.h"

#define USART5_RX_DATA_FRAME_LEN 18u
#define USART5_RX_BUF_LEN        18u

extern UART_HandleTypeDef huart5;

__attribute__((section(".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[USART5_RX_BUF_LEN];

__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);

static void uart5_rc_start_receive(void)
{
    HAL_UART_DMAStop(&huart5);
    __HAL_UART_CLEAR_FLAG(&huart5,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_IDLEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                 usart5_dma_rxbuf,
                                 USART5_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART5_Init(void)
{
    uart5_rc_start_receive();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != UART5)
    {
        return;
    }

    if ((HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) &&
        (size == USART5_RX_DATA_FRAME_LEN))
    {
        USART5_rxDataHandler(usart5_dma_rxbuf);
    }

    uart5_rc_start_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        uart5_rc_start_receive();
    }
}

__WEAK void USART5_rxDataHandler(uint8_t *rxBuf)
{
    (void)rxBuf;
}
