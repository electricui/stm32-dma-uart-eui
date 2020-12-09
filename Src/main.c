#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "electricui.h"

void SystemClock_Config(void);

#define STM32_UUID ((uint32_t *)0x1FFF7A10)

uint8_t   blink_enable  = 1;   //if the blinker should be running
uint8_t   led_state     = 0;   //track if the LED is illuminated
uint16_t  glow_time     = 200; //in milliseconds

// Timestamp used as part of LED timing behaviour
uint32_t led_timer = 0;

char nickname[15] = "STM32 DMA UART";   // human friendly device name

//void eui_serial_output(uint8_t *data, uint16_t len);

// Instantiate both interface management objects in an array
eui_interface_t eui_comm[] = {
        EUI_INTERFACE( &usart_send_buffer )
};

// Electric UI manages variables referenced in this array
eui_message_t tracked_vars[] =
{
    EUI_UINT8(  "led_blink",   blink_enable ),
    EUI_UINT8(  "led_state",   led_state ),
    EUI_UINT16( "lit_time",    glow_time ),
    EUI_CHAR_ARRAY_RO( "name", nickname ),
};

int main( void )
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    SystemClock_Config();

    MX_GPIO_Init();
    usart_init();       // setup of UART peripheral @ 115200 - DMA for RX (including idle-line interrupts) and DMA TX

    // Setup Electric UI
    EUI_LINK( eui_comm );
    EUI_TRACK( tracked_vars );
    eui_setup_identifier( (char *)STM32_UUID, 12 ); // 96-bit UUID is 12 bytes

    led_timer = HAL_GetTick();  //bootstrap the LED timer

    /*
     * The USART driver uses the ST LL hal to achieve DMA serial transfer in both directions.
     * DMA based transfers are ideal because they don't block the application and require the lowest overhead when
     * compared to polled or pure interrupt based approaches.
     *
     * In this example, circular buffers are used as the interface between user code and the raw DMA transaction buffers.
     * This allows DMA and UART interrupts (half-buffer, full buffer, idle rx) to quickly copy relevant data out
     * and let the user process the buffer in their own time.
    */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1)
    {
        // Check if the inbound buffer has any data in it
        while( usart_rx_buffer_ready() )
        {
            // Parse the inbound serial data
            eui_parse( usart_rx_buffer_read(), &eui_comm[0]);
        }

        // Simple blinking LED
        if( blink_enable )
        {
            // Check if the LED has been on for the configured duration
            if( HAL_GetTick() - led_timer >= glow_time )
            {
                eui_send_tracked("led_state");  // push the state to the UI for super-responsive graphs

                led_state = !led_state; //invert led state
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, led_state);   // set the orange LED
                led_timer = HAL_GetTick();

                eui_send_tracked("led_state");  // push the _new_ state to the graph for a crisp rising/falling edge
            }
        }

    }
#pragma clang diagnostic pop
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */