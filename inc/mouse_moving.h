#ifndef MY_MOVING_H
#define MY_MOVING_H
#define MmConvWheel 4096*4/1000/79
#define ONE_BLOCK 180
#define WheelDistance 69
extern int16_t left_speed,right_speed;
extern volatile uint8_t ENCODER_start;
void mouse_motor_setting();
void mouse_motor_moving();
void stop_motor();
void set_speed(int16_t left_speed,int16_t right_speed);
void set_left_motor(int16_t speed);
void set_right_motor(int16_t speed);
void encoder_setting();
void read_encoder();
void speed_controller(int16_t speed , float rad);
uint8_t set_param();
void go_straight();
void go_left();
void go_right();
void go_back();
void start_wall();
void turn_back();
int32_t len_measure(int32_t length);
void reset_e();
#endif
