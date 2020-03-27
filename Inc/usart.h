#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

/* ------------------------------------------------------------------*/

void
usart_init( void );

/* ------------------------------------------------------------------*/

size_t
usart_rx_buffer_ready( void );

/* ------------------------------------------------------------------*/

uint8_t
usart_rx_buffer_read( void );

/* ------------------------------------------------------------------*/

uint8_t
usart_send_string( const char* str );

uint8_t
usart_send_buffer( uint8_t* data, size_t len );

/* ------------------------------------------------------------------*/

void
usart_start_tx_dma( void );

/* ------------------------------------------------------------------*/

void
usart_rx_check( void );

/* ------------------------------------------------------------------*/

void
usart_process_data(const void* data, size_t len);



#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */