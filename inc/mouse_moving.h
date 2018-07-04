#ifndef MY_MOVING_H
#define MY_MOVING_H
#define ONE_BLOCK 180
#define WheelDistance 70.0 //69
#define WheelFromWall 40
extern volatile uint8_t ENCODER_start;
extern int16_t search_velocity;
extern float target_velocity;
extern float left_e_sum,right_e_sum;
extern float left_e,right_e;
extern float MmConvWheel;
extern int16_t left_speed,right_speed;
extern int16_t last_left_speed,last_right_speed;


void suction_motor_setting();
void suction_start(uint16_t po);
void suction_stop();

void mouse_motor_setting();
void mouse_motor_moving();
void stop_motor();
void set_speed(int16_t left_speed,int16_t right_speed);
void set_left_motor(int16_t velocity);
void set_right_motor(int16_t velocity);
void encoder_setting();
void read_encoder();
void speed_controller(int16_t speed , float rad);
uint8_t set_param();
void go_straight(float target_theta);
void go_left(int16_t degree);
void go_right(int16_t degree);
void go_back(float po);
void start_wall(int16_t po);
void turn_back(int16_t po);
void turn_side(int16_t target_direction,int8_t wall_dir);
int32_t len_measure(int32_t length);
void reset_e();
void start_withoutwall(int16_t po);
#endif
