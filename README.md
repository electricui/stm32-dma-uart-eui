# Bi-directional DMA UART with STM32F4 Discovery

This example firmware demonstrates using DMA for both rx and tx for minimal CPU load during transfers, and is 100% compatible with the [Electric UI Quickstart Tutorial](https://electricui.com/docs/quick-start/).

For a loopback example without Electric UI integration refer to [796ce88](https://github.com/Scottapotamas/stm32-dma-uart-eui/tree/796ce884e4cdb3793f36f6f9c88e0134bd459fc9).

## Why DMA?

DMA (Direct Memory Access) lets the microcontroller's peripherals transfer data to/from memory specified by the firmware developer, rather than making the developer read data out of registers manually.

DMA lets our USART/UART operate much efficiently than the more typical polling or per-byte interrupt based methods, as it seamlessly handles inbound and output transfers in the background with occasional interrupts when:

- the inbound buffer is half full,
- the inbound buffer is full,
- or the serial line has been idle for >1 frame!

This means the microcontroller isn't doing any work while data transfers, other than occasionally ensuring the buffer's don't overflow!

As a result, DMA transfers give one of the best UART experiences with Electric UI, with measured roundtrip heartbeat latency/jitter of `0.6ms ±0.2ms` at 115200 baud.
For reference, an Arduino Mega is about 6x slower, `4.5ms +-1.1ms`, for the same baudrate.

The actual time between the rx Idle Line interrupt firing at the end of the inbound packet, and the response packet's DMA transfer starting was measured around `0.65us +-0.21us`.

## Hardware Setup

- Hardware used was the ol' faithful green [STM32F4 Discovery Board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html), `STM32F407VGT6`.
- Connect an external USB-UART adaptor to `PD8`/`PD9` (and ground).
- The board is configured to run at `115200` baud by default.
- The orange led, `LD3` on `PD13`, is used as a blinker for the standard [`hello-electric` example](https://electricui.com/docs/quick-start/ui).

### Changing the microcontroller family

This example is setup against the `STM32F407VGT6`, but because it uses the ST HAL, changing to different F4 parts isn't too hard.

- If you're using `cmake`, change the `-DSTM32F407xx` to match your part i.e `-DSTM32F405xx`.
- Ensure your linker file `STM32F407BVGTx_FLASH.ld` file matches your micro, as your part might have more/less RAM and flash.
- Adjust the USART peripheral in use, GPIO pins, and LED pin as required for your development board.

## Firmware Setup

This project was created with CubeMX (.ioc file included), but the `usart.h/.c` are heavily generated. The project was exported for SW4STM32, but imported and edited with CLion.

We've used the ST `LL` hal libraries.

1. The `electricui-embedded` library isn't included in this repo, [download it here](https://github.com/electricui/electricui-embedded) and put the contents in `Drivers/electricui-embedded`.

    >You might need to delete or 'IDE exclude' the `electricui-embedded` sub-directories `test` and `examples` from the build.

2. Import the project into your IDE, or build using CLion's generated CMakeList.
3. Configure the build target as required
4. Build and flash the project to the devboard

Pre-built binaries for the `STM32F407VGT6 discovery board` are in `/binaries`.

## Acknowledgements

[Tilen Majerle's USART DMA](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx) examples are the best available resource for DMA USART on the STM32, this project basically combined several examples with some cleanup.