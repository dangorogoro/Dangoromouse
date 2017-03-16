#ifndef MY_USART_H
#define MY_USART_H
void USART_setting();
void USART_putc(USART_TypeDef* USARTx,char c);
void USART_puts(USART_TypeDef* USARTx,const char *s);
void USART_printf(const char *format,...);
void _sbrk_r();
void usart_write(uint16_t value);
#endif
