#include <stdio.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>

#define FLASH_BASE                      (0x00000000U)
#define BOOTLOADER_SIZE                 (0x8000U)
#define MAIN_APP_START_ADDRESS          (FLASH_BASE + BOOTLOADER_SIZE)

void jump_to_main_application (void);
void jump_to_main_application (void) {
    
    // Define the main application's vector table 
    vector_table_t* main_app_vector_table = (vector_table_t*)MAIN_APP_START_ADDRESS;

    // Use the reset function to jump to the main application
    main_app_vector_table->reset();
    
    
    
    
    /*// Programming magic to use a function pointer
    typedef void (*void_fn)(void);

    // Set up the pointer to the vector table of the main app
    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);

    // Tell the program that this function lives at the address in reset_vector
    void_fn jump_fn = (void_fn)reset_vector;

    // Jump to the main application
    jump_fn();
    */
}

int main(void)  {
    
    jump_to_main_application();

    // Never return
    return 0;
}