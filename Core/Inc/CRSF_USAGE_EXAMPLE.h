#ifndef __CRSF_USAGE_EXAMPLE_H__
#define __CRSF_USAGE_EXAMPLE_H__
/*
 * USAGE EXAMPLE: How to use AlfredoCRSF with STM32 platform abstraction
 *
 * C++-only! Do NOT include in C files or C translation units.
 */

#ifdef __cplusplus
#include "../../AlfredoCRSF/src/AlfredoCRSF.h"
#include "platform_abstraction.h"

// Create a UART stream and CRSF receiver instance
STM32Stream crsfSerial(&huart1);  // Use your UART handle
AlfredoCRSF crsf;

// In your initialization (e.g., user_init()):
inline void init_crsf() {
    crsf.begin(&crsfSerial);
    // Now crsf can be used normally
}

// In your main loop (e.g., user_loop_step()):
inline void update_crsf() {
    crsf.update();
    // Read channel values
    if (crsf.isLinkUp()) {
        int ch1 = crsf.getChannel(1);  // Get channel 1 in microseconds
        int ch2 = crsf.getChannel(2);
        // ... etc
    }
}
#endif // __cplusplus



/*
 * IMPORTANT: Make sure your UART is configured with:
 * - UART RX interrupt enabled (HAL_UART_Receive_IT)
 * - DMA disabled for RX (use interrupt mode instead)
 * - Interrupt priority not too high (allow platform_millis to work)
 * 
 * The platform_abstraction.cpp will automatically handle the
 * interrupt-driven receive via HAL_UART_RxCpltCallback.
 */

#endif // __CRSF_USAGE_EXAMPLE_H__
