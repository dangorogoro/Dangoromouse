#ifndef MY_SENSOR_H
#define MY_SENSOR_H
extern uint16_t led_1,led_2,led_3,led_4;
extern uint16_t sensor_sub;
void led_flash_setting();
void led_flash();
void led_get();
void led_stop();
uint8_t read_wall(uint8_t IndexDirection);
void sensor_works();
void reset_led();
void mouse_start();
extern volatile uint8_t SENSOR_start;
extern volatile uint8_t SENSOR_reset;
#endif
