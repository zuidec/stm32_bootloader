#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>

#include "core/common-defines.h"
#include "core/comms.h"
#include "core/uart.h"
#include "core/system.h"

//#define FLASH_BASE                      (0x00000000U)
#define BOOTLOADER_SIZE                 (0x8000U)
#define MAIN_APP_START_ADDRESS          (FLASH_BASE + BOOTLOADER_SIZE)

#define LED_PORT            (GPIOA)
#define LED_PIN             (GPIO0)

#define UART2_PORT          (GPIOA)
#define RX2_PIN             (GPIO3)
#define TX2_PIN             (GPIO2)

void jump_to_main_application (void);
void jump_to_main_application (void) {
    
    // Define the main application's vector table 
    vector_table_t* main_app_vector_table = (vector_table_t*)MAIN_APP_START_ADDRESS;

    // Use the reset function to jump to the main application
    main_app_vector_table->reset();
    
}

static void gpio_setup(void)    {
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Set gpio to alternate function, then set the alternate function to AF1 (TIM2)
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    //gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);

    // Set up UART2 and change the pins to alternate function
    gpio_mode_setup(UART2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX2_PIN | RX2_PIN );
    gpio_set_af(UART2_PORT,GPIO_AF7, TX2_PIN | RX2_PIN );
    
}

int main(void)  {
    
    gpio_setup();
    system_setup();
    //timer_setup();
    // Set up UART
    uart_setup();
    comms_setup();

    comms_packet_t packet = {
        .length = 9,
        .payload = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .crc = 0,
    };
    comms_packet_t rx_packet;
    packet.crc = comms_compute_crc(&packet);

    while(true) {
        
        comms_update();
        if(comms_packets_available())   {
            comms_read(&rx_packet);
        }
        comms_write(&packet);
        gpio_toggle(LED_PORT, LED_PIN);
        system_delay(500);
        
    }

    jump_to_main_application();

    // Never return
    return 0;
}