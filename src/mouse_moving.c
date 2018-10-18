#include "mine.h"
int16_t left_speed = 0,right_speed = 0;
int16_t last_left_speed = 0,last_right_speed = 0;
volatile uint8_t ENCODER_start = 0;
int16_t search_velocity = 600;
int16_t last_left_input = 0, last_right_input = 0;
float MmConvWheel = (4096.0 * 40.0 / 13.0 / 1000.0 / 77.0);  //79.0

float left_Kp = 5.17, right_Kp = 5.17; // 4.0 4.0
float left_Ki = 12.1, right_Ki = 12.1; //8.0 8.0
float left_Kd = 0.54, right_Kd = 0.54; //8.0 8.0
float rotate_Kp = 170.00;
float rotate_Ki = 218.68;
float rotate_Kd = 30.18;
//Kp = 4.06, Ki = 9.76, Kd = 0.389, T
//Kp = 0.561, Ki = 5.78, Kd = 0.00563
void suction_motor_setting(){
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse= 0;//look period
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1,ENABLE);
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
  TIM_Cmd(TIM1,ENABLE);
}
void suction_start(uint16_t po){
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = po; //to-1000
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; //to-1000
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void suction_stop(){
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;//look period

  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
}
void mouse_motor_setting(){
  //motor left OC1 OC2
  //motor right OC3 OC4
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse=10-1;//look period
  TIM_OC1Init(TIM3,&TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC2Init(TIM3,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC3Init(TIM3,&TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC4Init(TIM3,&TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3,ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_Cmd(TIM3,ENABLE);
}
void stop_motor(){
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
  TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
  TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
  TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
}
void set_left_motor(int16_t velocity){
  if(velocity >= 0){
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM3_Period - velocity - 1; //to-1000
    TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM3_Period - 1; //to-1000
    TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
  }
  else{
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM3_Period - 1; //to-1000
    TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM3_Period + velocity -1; //to-1000
    TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
  }
  last_left_input = velocity;
}
void set_speed(int16_t left_speed,int16_t right_speed){
  set_left_motor(-left_speed);
  set_right_motor(-right_speed);
}
void set_right_motor(int16_t velocity){
  if(velocity >= 0){
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM3_Period-1; //to-1000
    TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=TIM3_Period-velocity-1; //to-1000
    TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
  }
  else{
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=TIM3_Period+velocity-1; //to-100
    TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse=TIM3_Period-1; //to-100
    TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
  }
  last_right_input = velocity;
}
void encoder_setting(){
  TIM_Cmd(TIM2,ENABLE);
  TIM_Cmd(TIM8,ENABLE);
}
void read_encoder(){
  left_speed = TIM2->CNT;
  right_speed = TIM8->CNT;
  //if((mode_select % 10) != 2){
  TIM2->CNT = 0;
  TIM8->CNT = 0;
  //if(left_speed - last_left_speed > 150) left_speed = last_left_speed;
  //if(right_speed - last_right_speed > 150) right_speed = last_right_speed;
  len_counter += (left_speed + right_speed) / 2;
  //}
}
uint8_t set_param(){
  uint8_t value = (uint16_t)TIM_GetCounter(TIM2) / 1000 % 16;
  return value;
}

float left_e_sum = 0.f,right_e_sum = 0.f;
float left_e = 0.f,right_e = 0.f;
float left_e_old = 0.f,right_e_old=0.f;
float left_input = 0.f, right_input = 0.f;

float rad_e = 0.f;
float rad_e_sum = 0.f;
float rad_e_old = 0.f;
uint16_t left_speed_counter = 0, right_speed_counter = 0;
void speed_controller(int16_t target_speed,float target_rad){
  float GYRO_rad = 0;
  if(GYRO_start == ON){
    GYRO_rad = (float)(ReadGYRO()-GYRO_offset_data)/16.4/180.0*3.14;
    degree += GYRO_rad*180.0/3.14/1000.0;
    GYRO_start = OFF;
  }
  /*
     float left_target  = (float)((float)target_speed - target_rad * WheelDistance / 2.0);
     float right_target = (float)((float)target_speed + target_rad * WheelDistance / 2.0);
     */
  float left_target  = (float)target_speed;
  float right_target = (float)target_speed;
  left_e	=	(float)(left_target  - (float)(left_speed + right_speed) / 2.0f / (float)MmConvWheel);
  right_e	=	(float)(right_target - (float)(left_speed + right_speed) / 2.0f / (float)MmConvWheel);
  left_e_sum  += left_e / 1000.0;
  right_e_sum += right_e / 1000.0;

  rad_e = target_rad - GYRO_rad;
  rad_e_sum += rad_e / 1000.0;

  left_input = left_e * left_Kp + left_e_sum * left_Ki + left_Kd * (left_e - left_e_old);
  right_input = right_e * right_Kp + right_e_sum * right_Ki + right_Kd * (right_e - right_e_old);

  left_input -= rad_e * rotate_Kp + rad_e_sum * rotate_Ki + rotate_Kd * (rad_e - rad_e_old);
  right_input += rad_e * rotate_Kp + rad_e_sum * rotate_Ki + rotate_Kd * (rad_e - rad_e_old);

  if(left_input >= TIM3_Period){
    left_speed_counter++;
    if(left_speed_counter >= 500)left_input = 0;
    else  left_input = TIM3_Period - 2;
  }
  else  left_speed_counter = 0;
  if(right_input >= TIM3_Period){
    right_speed_counter++;
    if(right_speed_counter >= 500)right_input = 0;
    else  right_input = TIM3_Period - 2;
  }
  else right_speed_counter = 0;


  set_speed(left_input,right_input);
  left_e_old  = left_e;
  right_e_old = right_e;
  rad_e_old = rad_e;
  last_left_input = left_input;
  last_right_input = right_input;
}
void mouse_turn(const uint8_t value){
  while(1){
    degree = 0;
    set_speed(-25,25);
    if(value == 0){
      while(degree<87){
        if(GYRO_start==ON){
          GYRO_sampling();
          GYRO_start=OFF;
        }
      }
    }
    set_speed(0,0);
    stop_motor();
    degree=0;
    Delay_ms(1000);
  }
}
void go_straight(float po){
  float now_speed = (left_speed + right_speed) / 2.0 / MmConvWheel;
  float target_speed = now_speed;
  if(ENCODER_start == ON){
    read_encoder();
    if(target_speed < search_velocity - 30) target_speed += 30;
    else target_speed = search_velocity;
    //speed_controller(target_speed, -target_speed / 30.0 * po);
    speed_controller(target_speed, -15.0 * po);
    ENCODER_start = OFF;
  }
}
void turn_back(int16_t target_direction){
  int8_t turn_direction = target_direction  - 2;
  //turn_right
  float current_degree = degree;
  float first_turn_degree,second_turn_degree;
  set_speed(0,0);
  Delay_ms(100);
  reset_e();
  float last_rad = 8.f;
  float target_rad = 0.0;
  float rad_diff = 0.2f;
  bool first_flag = false;
  bool second_flag = false;
  while(degree >= turn_direction * 90){
    if(ENCODER_start == ON){
      read_encoder();
      if(target_rad > -last_rad && first_flag == false) target_rad -= rad_diff;
      else if(first_flag == false && target_rad <= -last_rad){
        target_rad = -last_rad;
        first_flag = true;
        first_turn_degree = degree;
        second_turn_degree = turn_direction * 90 + current_degree - first_turn_degree;
      }
      if(first_flag == true && degree <= second_turn_degree && second_flag == false)	second_flag = true;
      if(second_flag == true){
        target_rad += rad_diff;
        if(target_rad >= 0.0f)	break;
      }
      speed_controller(0,target_rad);
      ENCODER_start = OFF;
    }
  }
  len_counter = 0;
  set_speed(0,0);
  reset_e();
}
void turn_side(int16_t target_direction,int8_t wall_dir){
  float current_degree = degree;
  float first_turn_degree,second_turn_degree;
  set_speed(0,0);
  Delay_ms(100);
  reset_e();
  float last_rad = 6.f;
  float target_rad = 0.0;
  float rad_diff = 0.2f;
  bool first_flag = false;
  bool second_flag = false;
  if(wall_dir == 1){
    while(degree <= (target_direction + (int16_t)wall_dir) * 90){
      if(ENCODER_start == ON){
        read_encoder();
        if(target_rad < last_rad && first_flag == false) target_rad += rad_diff;
        else if(first_flag == false && target_rad >= last_rad){
          target_rad = last_rad;
          first_flag = true;
          first_turn_degree = degree;
          second_turn_degree = (target_direction + wall_dir) * 90 + current_degree - first_turn_degree;
        }
        if(first_flag == true && degree >= second_turn_degree && second_flag == false)	second_flag = true;
        if(second_flag == true){
          target_rad -= rad_diff;
          if(target_rad <= 0.0f)	break;
        }
        speed_controller(0,target_rad);
        ENCODER_start = OFF;
      }
    }
  }
  else{
    while(degree >= (target_direction + (int16_t)wall_dir) * 90){
      if(ENCODER_start == ON){
        read_encoder();
        if(target_rad > -last_rad && first_flag == false) target_rad -= rad_diff;
        else if(first_flag == false && target_rad <= -last_rad){
          target_rad = -last_rad;
          first_flag = true;
          first_turn_degree = degree;
          second_turn_degree = (target_direction + wall_dir) * 90 + current_degree - first_turn_degree;
        }
        if(first_flag == true && degree <= second_turn_degree && second_flag == false)	second_flag = true;
        if(second_flag == true){
          target_rad += rad_diff;
          if(target_rad >= 0.0f)	break;
        }

        speed_controller(0,target_rad);
        ENCODER_start = OFF;
      }
    }
  }
  set_speed(0,0);
  len_counter = 0;
  reset_e();
  Delay_ms(100);
}
void go_left(int16_t target_degree){
  float start_degree = degree;
  float last_rad = search_velocity / 50.0; //50 30 15
  float rad_size = last_rad / 20.0;
  float target_rad = 0;
  int8_t init_flag = 0;
  float first_degree = 0.0;
  while(degree <= target_degree){
    if(ENCODER_start == ON){
      read_encoder();
      if(init_flag == 0)	target_rad = (target_rad >= last_rad) ? last_rad : target_rad + rad_size;
      if(init_flag == 0 && target_rad >= last_rad){
        init_flag = 1;
        first_degree = degree;
      }
      if(init_flag == 1 && start_degree - first_degree <= degree - target_degree){
        if(target_rad >= 0.0) target_rad = target_rad - rad_size;
        else break;	
      }
      speed_controller(search_velocity,target_rad);
      ENCODER_start = OFF;
    }
  }
}
void go_right(int16_t target_degree){
  float start_degree = degree;
  float last_rad = search_velocity / 50.0;
  float rad_size = last_rad / 20.0;
  float target_rad = 0;
  int8_t init_flag = 0;
  float first_degree = 0.0;
  while(degree >= target_degree){
    if(ENCODER_start == ON){
      read_encoder();
      if(init_flag == 0)	target_rad = (target_rad >= last_rad) ? last_rad : target_rad + rad_size;
      if(init_flag == 0 && target_rad >= last_rad){
        init_flag = 1;
        first_degree = degree;
      }
      if(init_flag == 1 && start_degree - first_degree >= degree - target_degree){
        if(target_rad >= 0.0) target_rad = target_rad - rad_size;
        else break;	
      }
      speed_controller(search_velocity,- target_rad);
      ENCODER_start = OFF;
    }
  }
}

void go_back(float po){
  float now_speed = (left_speed + right_speed) / 2.0 / MmConvWheel;
  float target_speed = now_speed;
  if(ENCODER_start == ON){
    read_encoder();
    if(target_speed > -search_velocity / 2) target_speed -= 50;
    else target_speed = -search_velocity / 2;
    speed_controller(target_speed, -target_speed / 15.0 * po);
    ENCODER_start = OFF;
  }
}
void start_wall(int16_t po){
  set_speed(0,0);
  reset_e();
  len_counter = 0;
  uint16_t counter = 0;
  const threshold = 100;
  while(len_counter > len_measure(-50)){
    if(timer_clock == ON){
      counter += 1;
      timer_clock = OFF;
    }
    if(counter >= threshold){
      counter = 0;
      led_fulloff();
      pipi(2);
      pipi(3);
      pipi(5);
      break;
    }
    go_back(0);
  }
  degree = po * 90.0;
  reset_e();
  len_counter = 0;
  set_speed(0,0);
  Delay_ms(300);
  while(len_counter < len_measure(130)){
    const float target_theta =  (degree - po * 90.0) / 180.0 * PI;
    go_straight(target_theta);
  }
  len_counter = 0;
}
int32_t len_measure(int32_t length){
  return length * MmConvWheel * 1000;
}
void reset_e(){
  left_e_old  = 0;
  right_e_old = 0;
  left_e  = 0;
  right_e = 0;
  left_e_sum  = 0;
  right_e_sum = 0;

  rad_e = 0.f;
  rad_e_sum = 0.f;
  rad_e_old = 0.f;
}
void start_withoutwall(int16_t po){
  set_speed(0,0);
  reset_e();
  len_counter = 0;
  uint16_t counter = 0;
  const threshold = 300;
  while(len_counter > len_measure(-50)){
    go_back(0);
  }
  reset_e();
  len_counter = 0;
  set_speed(0,0);
  Delay_ms(300);
  while(len_counter < len_measure(150)){
    if(timer_clock == ON){
      counter += 1;
      timer_clock = OFF;
    }
    if(counter >= threshold){
      counter = 0;
      break;
    }
    const float target_theta =  (degree - po * 90.0) / 180.0 * PI;
    go_straight(target_theta);
  }
  len_counter = 0;
}
