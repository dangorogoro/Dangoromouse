#ifndef MY_SERVO_H
#define MY_SERVO_H
void set_servo();
void moving_servo(uint16_t angle,uint8_t mode);
void stop_servo(uint8_t mode);
void get_servo();
void release_servo();
void z_servo();
extern TIM_OCInitTypeDef TIM_OCInitStructure;
#endif
