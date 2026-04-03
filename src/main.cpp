#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define LED_DELAY_MS 250

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define STATUS_LED PICO_DEFAULT_LED_PIN

int main()
{
  stdio_init_all();
  uart_init(UART_ID, BAUD_RATE);

  gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
  gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

  gpio_init(STATUS_LED);
  gpio_set_dir(STATUS_LED, GPIO_OUT);

  uart_puts(UART_ID, "RPAUTO - CAN\r\n");
  uart_puts(UART_ID, "============\r\n");

  while(1)
  {
    gpio_put(STATUS_LED, true);
    sleep_ms(LED_DELAY_MS);
    gpio_put(STATUS_LED, false);
    sleep_ms(LED_DELAY_MS);
  }

  return 0;
}
