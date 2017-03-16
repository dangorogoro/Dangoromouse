#ifndef MY_SENSOR_H
#define MY_SENSOR_H
extern uint16_t led_1,led_2,led_3,led_4;
void led_flash_setting();
void led_flash();
void led_get();
uint8_t read_wall(uint8_t IndexDirection);
void sensor_works();
void reset_led();
extern volatile uint8_t SENSOR_start;
extern volatile uint8_t SENSOR_reset;
#endif
