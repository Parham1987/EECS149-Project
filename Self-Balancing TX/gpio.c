#include "gpio.h"


typedef struct{
	uint32_t GPIO_OUT;
	uint32_t GPIO_OUTSET;
	uint32_t GPIO_OUTCLR;
	uint32_t GPIO_IN;
	uint32_t GPIO_DIR;
	uint32_t GPIO_DIRSET;
	uint32_t GPIO_DIRCLR;
	uint32_t GPIO_LATCH;
	uint32_t GPIO_DETECTMODE;

}name1;
typedef struct{
	uint32_t PIN_CNF [32];
	
}name2;
name1 * first = (name1 *) 0x50000504;
name2 * second = (name2 *) 0x50000700;
// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	if(!dir){
		second->PIN_CNF[gpio_num] = (second->PIN_CNF[gpio_num] & 0xFFFFFFFE); 
	}
	else{
		//printf("GPIO CONFIG %lu\n", gpio_num);
		//printf("VALUE B4: %x\n", second->PIN_CNF[gpio_num]);
		second->PIN_CNF[gpio_num] = (second->PIN_CNF[gpio_num] & 0xFFFFFFFE) + 1;
		//printf("VALUE AFTER: %x\n", second->PIN_CNF[gpio_num]); 
	}
}

// Set gpio_num high
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	
	first->GPIO_OUT = first->GPIO_OUT | (1 << gpio_num);
}

// Set gpio_num low
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	//printf("GPIO NUM: %lu\n", gpio_num);
	//printf("GPIO CLEAR: %x\n", first->GPIO_OUT);
	first->GPIO_OUT = first->GPIO_OUT & ~(1 << gpio_num);
	//printf("GPIO CLEAR: %x\n", first->GPIO_OUT);
}

// Inputs: 
//  gpio_num - gpio number 0-31
int gpio_read(uint8_t gpio_num) {
    // should return pin state
    return first->GPIO_IN & (1 << gpio_num);
}
