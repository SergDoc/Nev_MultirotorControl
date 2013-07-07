
#include "board.h"

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/
#define UART_BUFFER_SIZE    512

// Receive buffer, circular DMA
volatile uint8_t rxBuffer[UART_BUFFER_SIZE];
volatile uint32_t rxDMAPos = 0;
volatile uint8_t txBuffer[UART_BUFFER_SIZE];
volatile uint32_t txBufferTail = 0;
volatile uint32_t txBufferHead = 0;
volatile bool txDMAEmpty = false;

static void uartTxDMA(void)
{
    //while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {
		//	};
		DMA2_Stream7->M0AR = (uint32_t)&txBuffer[txBufferTail];
    if (txBufferHead > txBufferTail) {
      //DMA_SetCurrDataCounter(DMA2_Stream7, txBufferHead - txBufferTail);   
			DMA2_Stream7->NDTR = txBufferHead - txBufferTail;
        txBufferTail = txBufferHead;
    } else {
      //DMA_SetCurrDataCounter(DMA2_Stream7, UART_BUFFER_SIZE - txBufferTail);  
			DMA2_Stream7->NDTR = UART_BUFFER_SIZE - txBufferTail;
        txBufferTail = 0;
    }
     txDMAEmpty = false;
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
  {
    /* Очищаем бит обработки прерывания */
     
    DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);
    DMA_Cmd(DMA2_Stream7, DISABLE);

    if (txBufferHead != txBufferTail)
			       uartTxDMA();
		else
        txDMAEmpty = true;
	}
		
}


void uartInit(uint32_t speed)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // USART1_TX    PA9
    // USART1_RX    PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer
    DMA_DeInit(DMA2_Stream5);
		while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE) {
			}; 
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    rxDMAPos = DMA_GetCurrDataCounter(DMA2_Stream5);

    // Transmit DMA
    DMA_DeInit(DMA2_Stream7);
		while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {
			}; 
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    DMA2_Stream7->NDTR = 0;
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

bool isUartAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA2_Stream5) != rxDMAPos) ? true : false;
}
bool isUartTransmitDMAEmpty(void)
		{
		    return txDMAEmpty;
		}

bool isUartTransmitEmpty(void)
{
    return (txBufferTail == txBufferHead);
}

uint8_t uartRead(void)
{
    uint8_t ch;

    ch = rxBuffer[UART_BUFFER_SIZE - rxDMAPos];
    // go back around the buffer
    if (--rxDMAPos == 0)
        rxDMAPos = UART_BUFFER_SIZE;

    return ch;
}

uint8_t uartReadPoll(void)
{
    while (!isUartAvailable()); // wait for some bytes
    return uartRead();
}

void uartWrite( uint8_t ch)
{
	  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {
			};
    txBuffer[txBufferHead] = ch;
	
    txBufferHead = (txBufferHead + 1) % UART_BUFFER_SIZE;

	
    // if DMA wasn't enabled, fire it up
    if (!(DMA2_Stream7->CR & 1))
        uartTxDMA();
}

void uartPrint(char *str)
{
    while (*str)
        uartWrite(*(str++));
}


/* -------------------------- UART2  ----------------------------- */
uartReceiveCallbackPtr uart2Callback = NULL;
#define UART2_BUFFER_SIZE    128

volatile uint8_t tx2Buffer[UART2_BUFFER_SIZE];
uint32_t tx2BufferTail = 0;
uint32_t tx2BufferHead = 0;
bool uart2RxOnly = false;

static void uart2Open(uint32_t speed)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | (uart2RxOnly ? 0 : USART_Mode_Tx);
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);


}

void uart2Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    uart2RxOnly = rxOnly;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2_TX    PD5
    // USART2_RX    PD6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    if (!rxOnly)
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6,GPIO_AF_USART2);

    uart2Open(speed);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if (!rxOnly)
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_Cmd(USART2, ENABLE);
		uart2Callback = func;
}

void uart2ChangeBaud(uint32_t speed)
{
    uart2Open(speed);
}

void uart2Write(uint8_t ch)
{
    if (uart2RxOnly)
        return;

    tx2Buffer[tx2BufferHead] = ch;
    tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

bool isUart2TransmitEmpty(void)
{
    return tx2BufferTail == tx2BufferHead;
}

void USART2_IRQHandler(void)
{
    uint16_t SR = USART2->SR;

    if (SR & USART_IT_RXNE) {
        if (uart2Callback)
            uart2Callback(USART_ReceiveData(USART2));
    }
    if (SR & USART_FLAG_TXE) {
        if (tx2BufferTail != tx2BufferHead) {
            USART2->DR = tx2Buffer[tx2BufferTail];
            tx2BufferTail = (tx2BufferTail + 1) % UART2_BUFFER_SIZE;
        } else {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
}

/* -------------------------- UART3  ----------------------------- */
uartReceiveCallbackPtr uart3Callback = NULL;
#define UART3_BUFFER_SIZE    128

volatile uint8_t tx3Buffer[UART3_BUFFER_SIZE];
uint32_t tx3BufferTail = 0;
uint32_t tx3BufferHead = 0;
bool uart3RxOnly = false;

static void uart3Open(uint32_t speed)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | (uart3RxOnly ? 0 : USART_Mode_Tx);
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void uart3Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    uart3RxOnly = rxOnly;

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2_TX    PD8
    // USART2_RX    PD9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    if (!rxOnly)
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,GPIO_AF_USART2);

    uart3Open(speed);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    if (!rxOnly)
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    USART_Cmd(USART3, ENABLE);
		uart3Callback = func;
}

void uart3ChangeBaud(uint32_t speed)
{
    uart3Open(speed);
}

void uart3Write(uint8_t ch)
{
    if (uart3RxOnly)
        return;

    tx3Buffer[tx3BufferHead] = ch;
    tx3BufferHead = (tx3BufferHead + 1) % UART3_BUFFER_SIZE;

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

bool isUart3TransmitEmpty(void)
{
    return tx3BufferTail == tx3BufferHead;
}

void USART3_IRQHandler(void)
{
    uint16_t SR = USART3->SR;

    if (SR & USART_IT_RXNE) {
        if (uart3Callback)
            uart3Callback(USART_ReceiveData(USART3));
    }
    if (SR & USART_FLAG_TXE) {
        if (tx3BufferTail != tx3BufferHead) {
            USART3->DR = tx3Buffer[tx3BufferTail];
            tx3BufferTail = (tx3BufferTail + 1) % UART3_BUFFER_SIZE;
        } else {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
