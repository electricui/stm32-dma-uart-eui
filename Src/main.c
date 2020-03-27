#include "main.h"
#include "usart.h"
#include "gpio.h"

void SystemClock_Config(void);

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

    /*
     * The USART driver uses the ST LL hal to achieve DMA serial transfer in both directions.
     * DMA based transfers are ideal because they don't block the application and require the lowest overhead when
     * compared to polled or pure interrupt based approaches.
     *
     * In this example, circular buffers are used as the interface between user code and the raw DMA transaction buffers.
     * This allows DMA and UART interrupts (half-buffer, full buffer, idle rx) to quickly copy relevant data out
     * and let the user process the buffer in their own time.
     *
     * This example uses a blocking delay for the LED blinker, and underlying UART transfers aren't affected,
     * but inbound data is only handled when we get time in the main loop (sometimes making the loopback wait until the blink is finished)
     *
     * For lowest latency responses/loopback, instead of polling the rx buffer as shown below, handling can be done
     * _in_ the interrupt chain by modifying usart_process_data().
     *
     * In a more structured program, we'd assume the micro is busy doing tasks (co-operative multitasking), or sleeping, in which case
     * the finished/idle interrupts would let us wake and process data with minimal latency.
     */

    usart_send_string("This is a DMA based RX/TX demo running on a STM32F407 Discovery Board\r\n");

    while (1)
    {
        // Check if the inbound buffer has any data in it
        while( usart_rx_buffer_ready() )
        {
            // Simple loopback between the rx and tx circular buffers
            uint8_t byte_loopback = usart_rx_buffer_read();
            usart_send_string( &byte_loopback );
        }

        // Simple state indication
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(20);
    }
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