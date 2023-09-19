#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>

#include "core/common-defines.h"
#include "core/comms.h"
#include "core/uart.h"
#include "core/system.h"
#include "core/simple-timer.h"
#include "core/bootloader-flash.h"


#define BOOTLOADER_SIZE                 (0x8000U)
#define MAIN_APP_START_ADDRESS          (FLASH_BASE + BOOTLOADER_SIZE)

#define DEFAULT_TIMEOUT     (10000)

#define LED_PORT            (GPIOA)
#define LED_PIN             (GPIO0)

#define UART2_PORT          (GPIOA)
#define RX2_PIN             (GPIO3)
#define TX2_PIN             (GPIO2)

#define DEVICE_ID           (0x42)
#define MAX_FW_LENGTH       ((1024U * 512U) - BOOTLOADER_SIZE)

typedef enum bl_state_t {
    BL_State_Sync,
    BL_State_WaitForUpdateReq,
    BL_State_DeviceIDReq,
    BL_State_DeviceIDRes,
    BL_State_FWLengthReq,
    BL_State_FWLengthRes,
    BL_State_EraseApplication,
    BL_State_Receive_FW,
    BL_State_Done,
} bl_state_t;

static bl_state_t state = BL_State_Sync;
static uint32_t fw_length = 0;
static uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};
static simple_timer_t timer;
static comms_packet_t temp_packet;

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
    // gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    //gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);

    // Set up UART2 and change the pins to alternate function
    gpio_mode_setup(UART2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX2_PIN | RX2_PIN );
    gpio_set_af(UART2_PORT,GPIO_AF7, TX2_PIN | RX2_PIN );
    
}

static void gpio_teardown(void)    {
    
    
    // Set gpio to alternate function, then set the alternate function to AF1 (TIM2)
    // gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    //gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);

    // Set up UART2 and change the pins to alternate function
    gpio_mode_setup(UART2_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, TX2_PIN | RX2_PIN );

    rcc_periph_clock_disable(RCC_GPIOA);
    
}

static void exit_bootloader(void) {
        state = BL_State_Done;
        comms_create_single_byte_packet(&temp_packet, BL_PACKET_NACK);
        comms_write(&temp_packet);
}

static void check_for_timeout(void) {
    if(simple_timer_has_elapsed(&timer)){
        exit_bootloader();
    }
}


static bool is_device_id_packet(const comms_packet_t* packet) {
    if(packet->length < 2)   {
        return false;
    }
    if(packet->payload[0]!=BL_PACKET_DEVICE_ID_RES_PAYLOAD)   {
        return false;
    }
    for (uint8_t i = 2; i < PACKET_PAYLOAD_LENGTH; i++) {
        if(packet->payload[i]!=0xff)    {
            return false;
        }
    }
    return true;
}

static bool is_fw_length_packet(const comms_packet_t* packet) {
    if(packet->length!=5)   {
        return false;
    }
    if(packet->payload[0]!=BL_PACKET_FW_LENGTH_RES_PAYLOAD)   {
        return false;
    }
    for (uint8_t i = 5; i < 5; i++) {
        if(packet->payload[i]!=0xff)    {
            return false;
        }
    }
    return true;
}

int main(void)  {
    
    gpio_setup();
    system_setup();
    //timer_setup();
    // Set up UART
    uart_setup();
    comms_setup();
    simple_timer_setup(&timer, DEFAULT_TIMEOUT, false);



    while(state != BL_State_Done) {
             if(state == BL_State_Sync)    {
                if(uart_data_available())   {
                    sync_seq[0] = sync_seq[1];
                    sync_seq[1] = sync_seq[2];
                    sync_seq[2] = sync_seq[3];
                    sync_seq[3] = uart_read_byte();

                    bool is_match = sync_seq[0] == SYNC_SEQ_0;
                    is_match = is_match && (sync_seq[1] == SYNC_SEQ_1);
                    is_match = is_match && (sync_seq[2] == SYNC_SEQ_2);
                    is_match = is_match && (sync_seq[3] == SYNC_SEQ_3);

                    if(is_match)    {
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_SYNC_OBSERVED_PAYLOAD);
                        comms_write(&temp_packet);
                        simple_timer_reset(&timer);
                        state = BL_State_WaitForUpdateReq;
                    }
                    else    {
                        check_for_timeout();
                    }
                }
                else {
                    check_for_timeout();
                }
                continue;
             }
             comms_update();
             switch (state) {
                    case BL_State_WaitForUpdateReq: {
                        if(comms_packets_available()) {
                            comms_read(&temp_packet);

                            if(comms_is_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_REQ_PAYLOAD))   {
                                comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_RES_PAYLOAD);
                                comms_write(&temp_packet);
                                simple_timer_reset(&timer);
                                state = BL_State_DeviceIDReq;
                            }
                            else{
                                check_for_timeout();
                            }
                        }
                        else{
                            check_for_timeout();
                        }
                        break;
                    }

                    case BL_State_DeviceIDReq:  {
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_DEVICE_ID_REQ_PAYLOAD);
                        comms_write(&temp_packet);
                        simple_timer_reset(&timer);
                        state = BL_State_DeviceIDRes;
                        break;
                    }

                    case BL_State_DeviceIDRes:  {
                        if(comms_packets_available()) {
                            comms_read(&temp_packet);

                            if(is_device_id_packet(&temp_packet)&& temp_packet.payload[1]==DEVICE_ID)   {
                                
                                simple_timer_reset(&timer);
                                state = BL_State_FWLengthReq;
                            }
                            else{
                                check_for_timeout();
                            }
                        }
                        else{
                            check_for_timeout();
                        }
                        break;
                    }

                    case BL_State_FWLengthReq:  {
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_LENGTH_REQ_PAYLOAD);
                        comms_write(&temp_packet);
                        simple_timer_reset(&timer);
                        state = BL_State_FWLengthRes;
                        break;
                    }

                    case BL_State_FWLengthRes:  {
                        if(comms_packets_available()) {
                            comms_read(&temp_packet);
                            fw_length = (
                                (temp_packet.payload[1])         |
                                (temp_packet.payload[2] << 8)    |
                                (temp_packet.payload[3] << 16)   |
                                (temp_packet.payload[4] << 24)   
                            );
                            if(is_fw_length_packet(&temp_packet) && fw_length <= MAX_FW_LENGTH)   {
                                simple_timer_reset(&timer);
                                state = BL_State_EraseApplication;
                            }
                            else{
                                check_for_timeout();
                            }
                        }
                        else{
                            check_for_timeout();
                        }
                        break;
                    }

                    case BL_State_EraseApplication: {
                        bootloader_flash_erase_main_application();

                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_READY_FOR_FW_PAYLOAD);
                        comms_write(&temp_packet);

                        simple_timer_reset(&timer);
                        state = BL_State_Receive_FW;
                        break;
                    }

                    case BL_State_Receive_FW:   {
                        if(comms_packets_available()) {
                            comms_read(&temp_packet);
                            
                            const uint8_t packet_length = (temp_packet.length & 0x0F) + 1;
                            bootloader_flash_write((MAIN_APP_START_ADDRESS + bytes_written), &temp_packet.payload[0], (uint32_t)packet_length);
                            bytes_written += packet_length;

                            if(bytes_written >= fw_length)  {
                                comms_create_single_byte_packet(&temp_packet, BL_PACKET_UPDATE_SUCCESSFUL_PAYLOAD);
                                comms_write(&temp_packet);
                                
                                state = BL_State_Done;
                            }
                            else{
                                comms_create_single_byte_packet(&temp_packet, BL_PACKET_READY_FOR_FW_PAYLOAD);
                                comms_write(&temp_packet);
                            }
                        }
                        else{
                            check_for_timeout();
                        }
                        break;
                    }

                    default:    {
                        state = BL_State_Sync;
                    }
             }
        
    }

    // Teardown peripherals from update
    system_delay(150);
    uart_teardown();
    system_teardown();
    gpio_teardown();

    // Jump to the main application
    jump_to_main_application();

    // Never return
    return 0;
}