#ifndef MY_MOVING_H
#define MY_MOVING_H
#define MmConvWheel (4096.0 * 44.0 /9.0 / 1000.0 / 75.0)  //79.0
#define ONE_BLOCK 180
#define WheelDistance 67.0 //69
#define WheelFromWall 20
extern int16_t left_speed,right_speed;
extern volatile uint8_t ENCODER_start;
extern uint16_t search_velocity;
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
void go_straight(float target_theta);
void go_left(int16_t degree);
void go_right(int16_t degree);
void go_back();
void start_wall(int16_t po);
void turn_back(int16_t po);
int32_t len_measure(int32_t length);
void reset_e();
#endif
