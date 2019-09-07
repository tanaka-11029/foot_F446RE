#include "mbed.h"
#include "neopixel.h"

const int wait_time[4][2] = {
    {6,14},
    {2, 5},
    {2, 6},
    {5,11}
};
    
NeoPixelOut::NeoPixelOut(PinName pin) : DigitalOut(pin)
{
    normalize = false;
    global_scale = 1.0f;
    #ifdef __STM32L432xx_H
        boad_ = 0;
    #endif
    #ifdef __STM32F446xx_H
        boad_ = 1;
    #endif
}

// The timing should be approximately 800ns/300ns, 300ns/800ns
void NeoPixelOut::byte(register uint32_t byte)
{        
    for (int i = 0; i < 8; i++) {
        gpio_write(&gpio, 1);
        
        // duty cycle determines bit value
        if (byte & 0x80) {
            // one
            for(int j = 0; j < wait_time[0][boad_]; j++) asm("NOP");//6 14
            
            gpio_write(&gpio, 0);
            for(int j = 0; j < wait_time[1][boad_]; j++) asm("NOP");//2 5
        }
        else {
            // zero
            for(int j = 0; j < wait_time[2][boad_]; j++) asm("NOP");//2 6
            
            gpio_write(&gpio, 0);
            for(int j = 0; j < wait_time[3][boad_]; j++) asm("NOP");//5 11
        }

        byte = byte << 1; // shift to next bit
    }
    
}

void NeoPixelOut::send(Pixel *colors, uint32_t count, bool flipwait)
{
    // Disable interrupts in the critical section
    __disable_irq();

    Pixel* rgb;
    float fr,fg,fb;
    for (int i = 0; i < count; i++) {
        rgb = colors++;
        fr = (int)rgb->r;
        fg = (int)rgb->g;
        fb = (int)rgb->b;
        
        if (normalize) {
            float scale = 255.0f/(fr+fg+fb);           
            fr *= scale;
            fg *= scale;
            fb *= scale;            
        }
        
        fr *= global_scale;
        fg *= global_scale;
        fb *= global_scale;
        
        if (fr > 255) fr = 255; 
        if (fg > 255) fg = 255;
        if (fb > 255) fb = 255;
        if (fr < 0) fr = 0; 
        if (fg < 0) fg = 0;
        if (fb < 0) fb = 0;
        
        // Black magic to fix distorted timing
        #ifdef __HAL_FLASH_INSTRUCTION_CACHE_DISABLE
        __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
        #endif
        
        byte((int)fg);
        byte((int)fr);
        byte((int)fb);
        
        #ifdef __HAL_FLASH_INSTRUCTION_CACHE_ENABLE
        __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
        #endif
    }

    __enable_irq();

    if (flipwait) flip();
}


void NeoPixelOut::flip(void)
{
    wait_us(50);    
}
