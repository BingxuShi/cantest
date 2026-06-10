

#include "ec11.h"
#include "usart.h"

unsigned int led_num = 0;
bool CW ;
bool CCW ;
unsigned int num_test = 0;


void EC11_update(void) {
    // static uint8_t lcd_long_press_active = 0;
    // static uint8_t lcd_button_pressed = 0;
    // if (READ(BTN_ENC) == 0)
    // { //button is pressed
    //     if (buttonBlanking.expired_cont(BUTTON_BLANKING_TIME)) {
    //         buttonBlanking.start();
    //         safetyTimer.start();
    //         if ((lcd_button_pressed == 0) && (lcd_long_press_active == 0))
    //         {
    //             longPressTimer.start();
    //             lcd_button_pressed = 1;
    //         }
    //         else if (longPressTimer.expired(LONG_PRESS_TIME))
    //         {
    //             lcd_long_press_active = 1;
    //             lcd_longpress_trigger = 1;
    //         }
    //     }
    // }
    // else
    // { //button not pressed
    //     if (lcd_button_pressed)
    //     { //button was released
    //         lcd_button_pressed = 0; // Reset to prevent double triggering
    //         if (!lcd_long_press_active)
    //         { //button released before long press gets activated
    //             lcd_click_trigger = 1; // This flag is reset when the event is consumed
    //         }
    //         lcd_backlight_wake_trigger = true; // flag event, knob pressed
    //         lcd_long_press_active = 0;
    //     }
    // }
    //manage encoder rotation
    static const unsigned int encrot_table[] = {
        0, -1, 1, 2,
        1, 0, 2, -1,
        -1, -2, 0, 1,
        -2, 1, -1, 0,
    };
    static int EC11_encoder_diff = 0;
    static unsigned int enc_bits_old = 0;
    unsigned int enc_bits = 0;

    if(!EC11_PH_A) 
        enc_bits |= _BV(0);
    if(!EC11_PH_B) 
        enc_bits |= _BV(1);
    
    if(enc_bits != enc_bits_old) {
        int8_t newDiff = encrot_table[(enc_bits_old << 2) | enc_bits];
        EC11_encoder_diff += newDiff;
        
        if (abs(EC11_encoder_diff) >= ENCODER_PULSES_PER_STEP) {
            if(EC11_encoder_diff < 0) {
                CW = true;
								HAL_UART_Transmit(&huart1, "zz", 2, 100);
            }
            if(EC11_encoder_diff > 0) {
                CCW = true;
								HAL_UART_Transmit(&huart1, "fz", 2, 100);
            }
            
        }
        if (abs(EC11_encoder_diff) >= ENCODER_PULSES_PER_STEP) {
            EC11_encoder_diff = 0;
        }
        enc_bits_old = enc_bits;
    }

}



