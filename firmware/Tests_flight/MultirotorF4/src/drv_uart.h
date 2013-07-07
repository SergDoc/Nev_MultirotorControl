#pragma once

// USART1
void uartInit(uint32_t speed);
bool isUartAvailable(void);
bool isUartTransmitEmpty(void);
bool isUartTransmitDMAEmpty(void);
uint8_t uartRead(void);
uint8_t uartReadPoll(void);
void uartWrite(uint8_t ch);
void uartPrint(char *str);

// USART2
void uart2Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly);
void uart2ChangeBaud(uint32_t speed);
bool isUart2TransmitEmpty(void);
void uart2Write(uint8_t ch);

// USART3 (GPS)
void uart3Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly);
void uart3ChangeBaud(uint32_t speed);
bool isUart3TransmitEmpty(void);
void uart3Write(uint8_t ch);
