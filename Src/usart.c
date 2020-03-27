/**
  * Based on other platform reference code at https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
  *
  */

#include "usart.h"
#include "ringbuff.h"


#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* ------------------------------------------------------------------*/

static ringbuff_t usart_tx_buff;
static uint8_t usart_tx_buff_data[1024];
static size_t usart_dma_tx_len;

static ringbuff_t usart_rx_dma_ringbuff;
static uint8_t usart_rx_dma_buffer[64]; // raw data for DMA
static uint8_t usart_rx_dma_ringbuff_data[128];

/* ------------------------------------------------------------------*/
// Setup the GPIO and USART peripheral at 115200 baud
// TX uses DMA1 Stream 3
// RX uses DMA1 Stream 1

void
usart_init(void)
{
    // Ring buffers need to be setup
    ringbuff_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));
    ringbuff_init(&usart_rx_dma_ringbuff, usart_rx_dma_ringbuff_data, sizeof(usart_rx_dma_ringbuff_data));

    // LL setup
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * USART3 GPIO Configuration
     *
     * PD8   ------> USART3_TX
     * PD9   ------> USART3_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init TX */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)&USART3->DR);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3); /* Enable TX TC interrupt */

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);



    /* USART3 DMA Init RX */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&USART3->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(usart_rx_dma_buffer));

    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);


    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);

    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_TX(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableIT_IDLE(USART3);

    /* USART interrupt */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    /* Enable USART */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);     // rx stream
    LL_USART_Enable(USART3);
}

/* ------------------------------------------------------------------*/
// Handle data from the RAW DMA buffer by putting it into a 'userspace' circular buffer
//
// Users can then query and grab bytes from the circular buffer in their own time
// without affecting underlying DMA

void
usart_process_data(const void* data, size_t len)
{
    // Commit the inbound data to the user-facing ring buffer
    ringbuff_write(&usart_rx_dma_ringbuff, data, len);
}

// Returns true if there is data in the rx ring buffer
size_t  usart_rx_buffer_ready(void)
{
    return ringbuff_get_full(&usart_rx_dma_ringbuff);
}

// Pop one byte off the rx ring buffer
// In more serious applications, blocks of data should be pulled based on available queue
uint8_t usart_rx_buffer_read(void)
{
    uint8_t byte = 0;
    ringbuff_read(&usart_rx_dma_ringbuff, &byte, 1);
    return byte;
}

/* ------------------------------------------------------------------*/
// DMA backed UART transmit copies the inbound data into a buffer, then starts DMA output

uint8_t
usart_send_string( const char* str )
{
    size_t len = strlen(str);
    uint8_t ret = 0;

    if( ringbuff_get_free(&usart_tx_buff) >= len )
    {
        ringbuff_write(&usart_tx_buff, str, len);
        usart_start_tx_dma();
        ret = 1;
    }

    return ret;
}

uint8_t
usart_send_buffer( uint8_t* data, size_t len )
{
    uint8_t ret = 0;

    if( ringbuff_get_free(&usart_tx_buff) >= len )
    {
        ringbuff_write(&usart_tx_buff, data, len);
        usart_start_tx_dma();
        ret = 1;
    }

    return ret;
}

// Provide the DMA peripheral with the ringbuffer output data
void
usart_start_tx_dma( void )
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    /* If transfer is not on-going */
    if (!LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_3))
    {
        usart_dma_tx_len = ringbuff_get_linear_block_read_length(&usart_tx_buff);

        /* Limit maximum size to transmit at a time */
        if (usart_dma_tx_len > 32)
        {
            usart_dma_tx_len = 32;
        }

        /* Anything to transmit? */
        if (usart_dma_tx_len > 0)
        {
            void* ptr = ringbuff_get_linear_block_read_address(&usart_tx_buff);

            /* Configure DMA */
            LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, usart_dma_tx_len);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)ptr);

            /* Clear all flags */
            LL_DMA_ClearFlag_TC3(DMA1);
            LL_DMA_ClearFlag_HT3(DMA1);
            LL_DMA_ClearFlag_DME3(DMA1);
            LL_DMA_ClearFlag_FE3(DMA1);
            LL_DMA_ClearFlag_TE3(DMA1);

            /* Start transfer */
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
        }
    }

    __set_PRIMASK(primask);
}

/* ------------------------------------------------------------------*/
// Tracks data handled by RX DMA and passes data off for higher-level storage/parsing etc.
// Called when the RX DMA interrupts for half or full buffer fire, and when line-idle occurs

void
usart_rx_check(void)
{
    static size_t previous_pos; // keep track of the position last time this function ran
    size_t current_pos;

    /* Calculate current position in buffer */
    current_pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

    if (current_pos != previous_pos)
    {
        /* Check change in received data */
        if (current_pos > previous_pos)
        {
            /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[previous_pos], current_pos - previous_pos);
        }
        else
        {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[previous_pos], ARRAY_LEN(usart_rx_dma_buffer) - previous_pos);

            /* Check and continue with beginning of buffer */
            if (current_pos > 0)
            {
                usart_process_data(&usart_rx_dma_buffer[0], current_pos);
            }
        }
    }
    previous_pos = current_pos;  /* Save current position as the new "previous" */

    /* Check and manually update if we reached end of buffer */
    if (previous_pos == ARRAY_LEN(usart_rx_dma_buffer))
    {
        previous_pos = 0;
    }
}

/* ------------------------------------------------------------------*/

void
DMA1_Stream1_IRQHandler( void )
{
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_HT1(DMA1))
    {
        LL_DMA_ClearFlag_HT1(DMA1);      /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);      /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other interrupt handling behaviour as needed */
}

void
DMA1_Stream3_IRQHandler(void)
{
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_3) && LL_DMA_IsActiveFlag_TC3(DMA1))
    {
        LL_DMA_ClearFlag_TC3(DMA1);             /* Clear transfer complete flag */
        ringbuff_skip(&usart_tx_buff, usart_dma_tx_len);/* Data sent, ignore these */
        usart_start_tx_dma();                   /* Try to send more data */
    }

    /* Implement other interrupt handling behaviour as needed */
}

void
USART3_IRQHandler(void)
{
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3))
    {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        usart_rx_check();                               /* Check for data to process */
    }

    /* Implement other interrupt handling behaviour as needed */
}

/* ------------------------------------------------------------------*/