//ISDT
#include "mine.h"
Maze maze,maze_backup;
uint16_t frontWallThreshold_3 = 2230, frontWallThreshold_4 = 2190; //3 was 2180
void Robot::setSpeed(){
  LeftEncoder = left_speed;
  RightEncoder = right_speed;
}
void Robot::setRobotVec(IndexVec vec){
  RobotVec = vec; 
}
void Robot::addRobotVec(IndexVec vec){
  RobotVec += vec;
}
void Robot::addRobotDirToVec(Direction dir){
  if(dir == NORTH)
    addRobotVec(IndexVec::vecNorth);
  else if(dir == WEST)
    addRobotVec(IndexVec::vecWest);
  else if(dir == SOUTH)
    addRobotVec(IndexVec::vecSouth);
  else if(dir == EAST)
    addRobotVec(IndexVec::vecEast);
}
IndexVec Robot::getRobotVec(){
  return RobotVec;
}
Direction Robot::getRobotDir(){
  return RobotDir;
}
void Robot::setRobotDir(Direction dir){
  RobotDir = dir;
}
void Robot::set_coordinate(float coordinate_x, float coordinate_y){
  float speed = (left_speed + right_speed) / 2.0 / MmConvWheel / 1000.0;
  set_x(speed * coordinate_x);
  set_y(speed * coordinate_y);
  set_vr_x(speed * coordinate_x);
  set_vr_y(speed * coordinate_y);
}
void Robot::add_coordinate(float degree){
  float curent_speed = (left_speed + right_speed) / 2.0 / MmConvWheel / 1000.0;
  float coordinate_x = -sin(degree * PI / 180.0);
  float coordinate_y = cos(degree * PI / 180.0);
  float x_input = curent_speed * coordinate_x;
  float y_input = curent_speed * coordinate_y;
  add_x(x_input);
  add_y(y_input);
  add_vr_x(x_input);
  add_vr_y(y_input);
}
void Robot::add_coordinate(float degree, float slip_rad){
  float latest_rad = degree * PI / 180.0;
  float real_rad = latest_rad - slip_rad;
  float curent_speed = (left_speed + right_speed) / 2.0 / MmConvWheel / 1000.0;
  add_x(curent_speed * -sin(latest_rad));
  add_y(curent_speed * cos(latest_rad));
  add_vr_x(curent_speed * -sin(real_rad));
  add_vr_y(curent_speed * cos(real_rad));
  set_x(getPosition().vr_x);
  set_y(getPosition().vr_y);
}
float Robot::centerDistance(){
  float distance = 0.0f;
  if(RobotRunVec(0,0) != 0)
    distance =  RobotRunVec(0,0) * (RobotVec.y * 180.0  + 50.0 - y()); //50.0
  else	distance = - RobotRunVec(0,1) * (RobotVec.x * 180.0 - x());
  return distance;
}
void Robot::startOffSet(Agent *agent){
  sensor_sub = 0;
  for(int i=0;i<10;i++){
    reset_led();
    sensor_works();
    sensor_sub += led_2 - led_1;
    led_2_reference += led_2;
    led_1_reference += led_1;
  }
  sensor_sub /= 10;
  led_1_reference /= 10;
  led_2_reference /= 10;
  //led_1_reference = 2316;
  //led_2_reference = 2268;
  agent->update(Robot::getRobotVec(),0b11111110);
  while(len_counter <= len_measure(130)){
    go_straight(0);
  }
  Robot::addRobotVec(IndexVec::vecNorth);
  len_counter = 0;
}
float Robot::runningCoordinate(){
  if(RobotRunVec(0,0) != 0) return x();
  else return y();
}

void Robot::setRobotVecFromRun(uint8_t dir, uint8_t n){
  Matrix2i nextDir;
  if((dir != Operation::TURN_RIGHT135) && (dir != Operation::TURN_LEFT135) && (dir != Operation::TURN_RIGHT45) && (dir != Operation::TURN_LEFT45)){
    if(dir == Operation::FORWARD)
      nextDir = eigenRotate();
    else if((dir == Operation::TURN_RIGHT90) || (dir == Operation::TURN_RIGHT90S))
      nextDir = right90Rotate();
    else if((dir == Operation::TURN_LEFT90) || (dir == Operation::TURN_LEFT90S))
      nextDir = left90Rotate();
    RobotRunVec = RobotRunVec * nextDir;
    IndexVec NextState(n * RobotRunVec(0,0),n * RobotRunVec(0,1));
    RobotVec += NextState;
  }
}
void Robot::goStraight(){
  goStraight(ONE_BLOCK);
}
/*
   void Robot::goStraight(){
   while(len_counter <= len_measure(ONE_BLOCK)){
   float wall_value = 0.0f;
   if(SENSOR_reset == ON){
   start_buzzer(10);
   if(sideWall == true && led_1 >= led_1_threshold && led_2 >= led_2_threshold) wall_value = (led_2 - led_1 - sensor_sub) / 25.0;
   else if(leftWall	== true && led_1 >= led_1_threshold) wall_value = (-led_1 + led_1_reference) / 15.0;
   else if(rightWall	== true && led_2 >= led_2_threshold) wall_value = (led_2 - led_2_reference) / 15.0;
   reset_led();
   SENSOR_reset = OFF;
   }
   if(SENSOR_start == ON)	led_get();
   float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI - wall_value;
   go_straight(target_theta); // *- 15
   }
   }*/
void Robot::goStraight(uint16_t length){
  while(len_counter <= len_measure(length)){
    float wall_value = 0.0f;
    if(SENSOR_reset == ON){
      start_buzzer(10);
      if(sideWall == true && led_1 >= led_1_threshold && led_2 >= led_2_threshold) wall_value = (led_2 - led_1 - sensor_sub) / 25.0; // 15
      else if(leftWall	== true && led_1 >= led_1_threshold) wall_value = (-led_1 + led_1_reference) / 15.0;
      else if(rightWall	== true && led_2 >= led_2_threshold) wall_value = (led_2 - led_2_reference) / 15.0;
      reset_led();
      SENSOR_reset = OFF;
    }
    if(SENSOR_start == ON)	led_get();
    float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI - wall_value;
    go_straight(target_theta);
  }
}
void Robot::slalomStraight(uint16_t length){
  if(frontWall == true){
    bool frontThreshold = false;
    while(frontThreshold == false){
      if(SENSOR_reset == ON){
        if(led_3 >= frontWallThreshold_3 && led_4 >= frontWallThreshold_4) frontThreshold = true; 
        reset_led();
        SENSOR_reset = OFF;
      }
      if(SENSOR_start == ON)	led_get();
      float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI;
      go_straight(target_theta);
    }
  }
  else	goStraight(length);
}
void Robot::goRight(){

  reset_led();
  uint16_t offset = 40;
  len_counter = 0;
  slalomStraight(offset);
  len_counter = 0;
  addRobotDegreeDir(-1);
  go_right(getRobotDegreeDir() * 90 );
  len_counter = 0;
  goStraight(offset);
}
void Robot::goLeft(){
  reset_led();
  uint16_t offset = 40;
  len_counter = 0;
  slalomStraight(offset);
  len_counter = 0;
  addRobotDegreeDir(1);
  go_left(getRobotDegreeDir() * 90);
  len_counter = 0;
  goStraight(offset);
}
void Robot::startBack(Direction target_dir, bool reverse_flag){
  setWallStatus();
  len_counter = 0;
  if(reverse_flag == 0)	goStraight(40);
  int32_t back_length = -40;
  int8_t wall_dir = 0; // 1 right -1 left
  if(leftWall == true) wall_dir = -1;
  else if(rightWall == true) wall_dir = 1;
  ////////////////////////////////////////write here!!!!!!!!!!!!!!!!!!!!!!!!
  len_counter = 0;
  set_speed(0,0);
  reset_e();
  Delay_ms(300);
  uint16_t counter = 0;
  const uint16_t threshold = 50;

  if(wall_dir != 0){
    turn_side(getRobotDegreeDir(),wall_dir);
    addRobotDegreeDir(wall_dir);
    while(len_counter > len_measure(back_length)){
      if(timer_clock == ON){
        counter += 1;
        timer_clock = OFF;
      }
      if(counter >= threshold){
        counter = 0;
        break;
      }
      const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
      go_back(-target_theta);
    }
    degree = getRobotDegreeDir() * 90.0 ;
    reset_e();
    len_counter = 0;
    set_speed(0,0);
    Delay_ms(100);
    while(len_counter < len_measure(20)){
      const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
      go_straight(target_theta);
    }
    set_speed(0,0);
    reset_e();
    Delay_ms(100);
    turn_side(getRobotDegreeDir(),wall_dir);
    addRobotDegreeDir(wall_dir);
  }
  else{
    turn_back(getRobotDegreeDir());
    setRobotDegreeDir(0);
    degree = getRobotDegreeDir() * 90.0;
  }

  len_counter = 0;
  while(len_counter > len_measure(back_length)){
    if(timer_clock == ON){
      counter += 1;
      timer_clock = OFF;
    }
    if(counter >= threshold){
      counter = 0;
      break;
    }
    const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
    go_back(-target_theta);
  }
  reset_e();
  degree = 0;
  set_speed(0,0);
  len_counter = 0;
  setRobotDegreeDir(target_dir);
}
void Robot::goBack(int8_t Nextdir, bool goal_flag = false){
  goStraight(60);
  int8_t wall_dir = 0; // 1 right -1 left
  if(leftWall == true) wall_dir = -1;
  else if(rightWall == true) wall_dir = 1;
  ////////////////////////////////////////write here!!!!!!!!!!!!!!!!!!!!!!!!
  len_counter = 0;
  set_speed(0,0);
  reset_e();
  Delay_ms(300);
  uint8_t searching_save_flag = getSearchingSaveFlag();
  if(true == getSaveMazeFlag() || 0 == searching_save_flag){
    save_mazedata(maze);
    pipi(2);
    pipi(3);
    pipi(5);
  }
  int8_t value;
  uint16_t counter = 0;
  const uint16_t threshold = 50;

  if(Nextdir == NORTH)	value = 0;
  if(Nextdir == WEST)	value = 1;
  if(Nextdir == EAST)	value = -1;
  if(Nextdir == SOUTH)	value = 2;
  if(wall_dir != 0){
    turn_side(getRobotDegreeDir(),wall_dir);
    addRobotDegreeDir(wall_dir);
    while(len_counter > len_measure(-30)){
      if(timer_clock == ON){
        counter += 1;
        timer_clock = OFF;
      }
      if(counter >= threshold){
        counter = 0;
        led_fulloff();
        break;
      }
      const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
      go_back(-target_theta);
    }
    if(frontWall == false)	setRobotDegreeDir(value - wall_dir);
    degree = getRobotDegreeDir() * 90.0 ;
    reset_e();
    len_counter = 0;
    set_speed(0,0);
    Delay_ms(300);

    if(false == goal_flag){
      while(len_counter < len_measure(40)){
        const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
        go_straight(target_theta);
      }
      set_speed(0,0);
      reset_e();
      Delay_ms(100);
      turn_side(getRobotDegreeDir(),wall_dir);
      addRobotDegreeDir(wall_dir);
    }
  }
  else{
    turn_back(getRobotDegreeDir());
    setRobotDegreeDir(value);
    degree = getRobotDegreeDir() * 90.0;
  }
  if(wall_dir != 0 && frontWall == true)	setRobotDegreeDir(value);
  reset_e();

  if(false == goal_flag && frontWall == false)	start_withoutwall(getRobotDegreeDir());
  else start_wall(getRobotDegreeDir());
  //Not consider wall around goal seriously (front)
}
void Robot::setWallStatus(){
  if(led_1 >= led_1_threshold)	leftWall = true;
  else leftWall = false;
  if(led_2 >= led_2_threshold)	rightWall = true;
  else rightWall = false;
  if(led_1 >= led_1_threshold && led_2 >= led_2_threshold)	sideWall = true;
  else sideWall = false;
  if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)	frontWall = true;
  else frontWall = false;
}
void Robot::robotMove(Direction Nextdir, bool goal_flag = false){
  setWallStatus();
  len_counter = 0;
  Direction Nowdir = Robot::getRobotDir();
  if(true == goal_flag && (leftWall == false || rightWall == false))  goBack(Nextdir,true);
  else if(Nowdir == NORTH){
    if(Nextdir == NORTH){
      goStraight();
    }
    if(Nextdir == EAST){
      goRight();
    }
    if(Nextdir == WEST){
      goLeft();
    }
    if(Nextdir == SOUTH){
      goBack(Nextdir);
    }
  }
  else if(Nowdir == WEST){
    if(Nextdir == NORTH){
      goRight();
    }
    if(Nextdir == EAST){
      goBack(Nextdir);
    }
    if(Nextdir == WEST){
      goStraight();
    }
    if(Nextdir == SOUTH){
      goLeft();
    }
  }
  else if(Nowdir == SOUTH){
    if(Nextdir == NORTH){
      goBack(Nextdir);
    }
    if(Nextdir == EAST){
      goLeft();
    }
    if(Nextdir == WEST){
      goRight();
    }
    if(Nextdir == SOUTH){
      goStraight();
    }
  }
  else if(Nowdir == EAST){
    if(Nextdir == NORTH){
      goLeft();
    }
    if(Nextdir == EAST){
      goStraight();
    }
    if(Nextdir == WEST){
      goBack(Nextdir);
    }
    if(Nextdir == SOUTH){
      goRight();
    }
  }
}
float target_degree = 0.0f;
int16_t now_speed = 0;
float slip_k = 2.5e-5;//4.1e-5
//float slip_k = 0.00009;
void Robot::robotShortMove(OperationList root,Param param,size_t *i){
  if(*i == 0){
    target_degree = 0.0f;
    now_speed = 0;
    left_speed = 0;
    right_speed = 0;
  }
  else	now_speed = (left_speed + right_speed) / 2 / MmConvWheel;

  len_counter = 0;
  //uint16_t curving_length = param.get_turn_param() / 30; // 30
  int16_t curving_length = 0;
  if(root[(*i)+1].op == Operation::TURN_LEFT90S || root[(*i)+1].op == Operation::TURN_RIGHT90S) curving_length = 0;

  if(root[(*i)].op != Operation::STOP){
    if(root[(*i)+1].op == Operation::STOP) curving_length += 90;
  }
  //int16_t last_speed = (root[(*i)].n == 1) ? param.get_turn_param() :param.get_last_param();
  int16_t last_speed = param.get_last_param();
  int16_t turn_speed;
  if(root[(*i)+1].op == Operation::TURN_RIGHT90S || root[(*i)+1].op == Operation::TURN_LEFT90S)	turn_speed = param.get_small_turn_param();
  else if(root[(*i)].op == Operation::TURN_RIGHT90S || root[(*i)].op == Operation::TURN_LEFT90S)	turn_speed = param.get_small_turn_param();
  else turn_speed = param.get_turn_param();
  const int16_t accel = param.get_accel_param();
  int16_t length = 0;
  curving_length = (root[(*i)+1].op == Operation::TURN_RIGHT45 || root[(*i)+1].op == Operation::TURN_LEFT45) ? 90 - 40 : curving_length;
  curving_length = (root[(*i)+1].op == Operation::TURN_RIGHT135 || root[(*i)+1].op == Operation::TURN_LEFT135) ? 90 - 40 : curving_length;
  curving_length = (root[(*i)+1].op == Operation::TURN_RIGHT180 || root[(*i)+1].op == Operation::TURN_LEFT180) ? 90 - 50 : curving_length;
  curving_length = (root[(*i)+1].op == Operation::TURN_RIGHT90 || root[(*i)+1].op == Operation::TURN_LEFT90) ? 90 - 50 : curving_length;
  length = *i == 0 ? 138 + (root[*i].n - 1) * ONE_BLOCK - curving_length : root[*i].n * ONE_BLOCK - curving_length; //130 was
  if(*i > 0){
    length = (root[(*i)-1].op == Operation::TURN_RIGHT45 || root[(*i)-1].op == Operation::TURN_LEFT45) ? length - (90 - 40) : length;
    length = (root[(*i)-1].op == Operation::TURN_RIGHT135 || root[(*i)-1].op == Operation::TURN_LEFT135) ? length - (90 - 40) : length;
    length = (root[(*i)-1].op == Operation::TURN_RIGHT90 || root[(*i)-1].op == Operation::TURN_LEFT90) ? length - (90 - 50) : length;
    length = (root[(*i)-1].op == Operation::TURN_RIGHT180 || root[(*i)-1].op == Operation::TURN_LEFT180) ? length - (90 - 50) : length;
  }


  float e_now = 0, e_sum = 0;
  float target_theta_now = 0,target_theta_last = 0;
  float x_p = 1.0 / 600.0 * last_speed;
  float x_i = 1.0 / 600.0 * last_speed;
  float degree_p = 13.0;
  float degree_i = 1.5;
  float degree_d = 1.0;
  float sensor_p = 0.7;
  //float diagKx = 0.0003;//151520
  //float diagKy = 4.00;
  //float diagKtheta = 0.0015;
  //float diagKx = 0.0004;//0.00055
  //float diagKxD = 0.002;
  //float diagKy = 1.50;
  //float diagKtheta = 0.0025;//0.008
  //float diagKthetaD = 0.003;
  float diagKx = 0.0003;//0.00055
  float diagStraightKx = 0.0001;//0.00055
  float diagKxD = 0.002;
  float diagKy = 1.50;
  float diagKtheta = 0.0025;//0.008
  float diagKthetaD = 0.003;
  float slip_rad = 0.0;
  //setRobotVecFromRun(root[*i].op,root[*i].n);
  Direction latestDirection = directionFromRunVec(RobotRunVec);
  if(root[*i].op == Operation::FORWARD){
    setRobotVecFromRun(root[*i].op,root[*i].n);
    sensor_works();
    set_left_sensor(led_1);
    set_right_sensor(led_2);
    reset_led();
    uint8_t prescaler = 0;
    float target_theta_sum = 0;

    stop_buzzer();
    led_fullon();
    float left_value, right_value;
    float last_left_value, last_right_value;
    float last_len = 0;
    bool last_wall = false;
    bool last_wall_check = false;
    float last_e_x = 0.0, last_e_theta = 0.0;
    while(last_wall == true || judgeTargetCoordinate(getRobotVec(),RobotRunVec,curving_length)){
      float wall_value = 0.0f;
      x_p = 1.0 / 600.0 * now_speed;
      x_i = 1.0 / 600.0 * now_speed;
      if(SENSOR_reset == ON){
        left_value = led_1;
        right_value = led_2;
        if(led_1 >= led_1_threshold || led_2 >= led_2_threshold ){
          if(led_1 >= led_1_threshold && led_2 >= led_2_threshold)
            wall_value = (led_2 - led_1 - sensor_sub) / 50.0;
          else if(led_1 >= led_1_threshold)
            wall_value = (-led_1 + led_1_reference) / 300.0;
          else if(led_2 >= led_2_threshold)
            wall_value = (led_2 - led_2_reference) / 300.0;
          led_fullon();
          if(traject_clock == ON){
            traject_clock = OFF;

            if(fabs(degree - target_degree) < 2.0 && len_counter > len_measure(30))  fixCoordinate(RobotRunVec, led_1, led_2);
            /*
            if(led_1 >= led_1_threshold && led_2 >= led_2_threshold && abs(led_1 - led_2) < 250)  fixCoordinate(RobotRunVec, led_1, led_2);
            else if(led_1 >= (led_1_threshold + 150) && led_2 < led_2_threshold) fixCoordinate(RobotRunVec, 2 * (led_1 - led_1_reference), 0);
            else if(led_2 >= (led_2_threshold + 150) && led_1 < led_1_threshold) fixCoordinate(RobotRunVec, 0, 2 * (led_2 - led_2_reference));
            */
          }
          /*
          if(len_counter >= len_measure(length - 150) && last_wall_check == false){
            if((root[(*i)+1].op == Operation::TURN_RIGHT45 || root[(*i)+1].op == Operation::TURN_RIGHT90 || root[(*i)+1].op == Operation::TURN_RIGHT135) && led_2 > led_2_threshold)
              last_wall = true;
            else if((root[(*i)+1].op == Operation::TURN_LEFT45 || root[(*i)+1].op == Operation::TURN_LEFT90 || root[(*i)+1].op == Operation::TURN_LEFT135) && led_1 > led_1_threshold)
              last_wall = true;
            else last_wall_check = true; // dont find wall
          }
          */
        }
        else led_fulloff();
        reset_led();
        SENSOR_reset = OFF;
      }
      float value = 0;
      if(SENSOR_start == ON)	led_get();

      if(ENCODER_start == ON){
        if(checkZAccel()){
          zStatus = true;
          return;
        }
        if(len_counter >= len_measure(length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)turn_speed * (int32_t)turn_speed) / 2.0 / (accel * 1000))) //conv to mm
          now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
        else
          now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
        read_encoder();
        add_coordinate(degree);

        Position targetPos = estimatePosition(getPosition());
        float target_offset = 25.0;
        if(latestDirection == NORTH) targetPos.y = y() + target_offset;
        else if(latestDirection == SOUTH) targetPos.y = y() - target_offset;
        else if(latestDirection == EAST) targetPos.x = x() + target_offset;
        else if(latestDirection == WEST) targetPos.x = x() - target_offset;
        float e_x = targetPos.x - x();
        float e_y = targetPos.y - y();
        float tmp = e_x;
        float threshold = 15.0;
        e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
        e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);

        
        if(e_x > threshold) e_x = threshold;
        else if(e_x < -threshold) e_x = -threshold;
        target_theta_now = (degree - target_degree) / 180.0 * PI;
        target_theta_sum += target_theta_now / 1000.0;
        float target_theta_diff = target_theta_now - target_theta_last;
        target_theta_last = target_theta_now;
        float theta_input = -(degree_d * target_theta_diff + degree_i * target_theta_sum + degree_p * target_theta_now);
        //value = - (degree_d * target_theta_diff + degree_i * target_theta_sum + degree_p * target_theta_now - sensor_p * wall_value)/*+ (-target_theta_now + target_theta_last) * degree_d) + e_now * x_p + e_sum * x_i*/;
        value = theta_input + (sensor_p * wall_value);
        //speed_controller(now_speed, value);
        //speed_controller(now_speed * cos(target_theta_now) /*+ diagKy * e_y*/,);
        //speed_controller(now_speed * cos(-target_theta_now) + diagKy * e_y, now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin(-(target_theta_now - last_e_theta)) + diagKtheta * sin(-target_theta_now)));
        speed_controller(now_speed * cos(-target_theta_now) + diagKy * e_y, now_speed * (-(diagStraightKx * e_x + diagKxD * (e_x - last_e_x))) + theta_input);
        last_e_x = e_x;
        last_e_theta = target_theta_now;
        ENCODER_start = OFF;
      }
      if(timer_clock == ON){
        prescaler = (prescaler + 1) % 100;
        timer_clock = OFF;
        //plot.push_back(x(), y(), left_speed / MmConvWheel, right_speed / MmConvWheel, left_input, right_input, now_speed, left_value, right_value, last_left_value, last_right_value);
        //plot.push_back(x(), y(),0,0,0,0);
        Position tmp = getPosition();
        plot.push_back(tmp.x, tmp.y, 0,0, tmp.vr_x, tmp.vr_y);
        //if(prescaler % 2 == 0){
      }
      if(wall_detect == ON){
        if(last_left_value > 2048 && last_right_value > 2048 && left_value > 2048 && right_value > 2048 && ((fabs(last_right_value - right_value) > 50) || (fabs(last_left_value - left_value) > 50)) && ((left_value < led_1_threshold || last_left_value < led_1_threshold) || (right_value < led_2_threshold || last_right_value < led_2_threshold)) && (len_counter - last_len) > len_measure(30)){
          /*
          if(last_wall == true){
            last_wall = false;
            last_wall_check = true;
          }
          */
          last_len = len_counter;
          led_fulloff();
          float fix_length = 30.0;
          float offset_length = 0.0;
          //if(right_value - last_right_value > 60 || left_value - last_left_value > 60) fix_length += offset_length;
          if(last_right_value - right_value > 50 || last_left_value - left_value > 50) fix_length -= offset_length;;
          fixCoordinate(RobotRunVec, -fix_length);
          start_buzzer(7);
        }
        last_left_value = left_value;
        last_right_value = right_value;
        set_left_sensor(led_1);
        set_right_sensor(led_2);
        wall_detect = OFF;
      }
    }
  }
  else if(root[*i].op == Operation::STOP){
    led_fulloff();
    set_speed(0,0);
  }
  else if(judge_diag_turn(root[*i].op)){
    Operation::OperationType turn_type = root[*i].op;
    Position centerPosition = getPositionFromVec(getRobotVec(), directionFromRunVec(RobotRunVec));
    /*
    Position centerPosition;// = estimatePosition(get_position());
    centerPosition.x = x();
    centerPosition.y = y();
    */
    Matrix2i operateRunVec = RobotRunVec;
    Matrix2i firstRunVec = RobotRunVec;
    Matrix2i nextRunVec = RobotRunVec;
    Direction firstDir = directionFromRunVec(RobotRunVec);
    Direction nextDir = directionFromRunVec(RobotRunVec);
    if(firstDir == NORTH)  centerPosition.y -= 90;
    else if(firstDir== SOUTH)  centerPosition.y += 90;
    else if(firstDir== EAST)  centerPosition.x -= 90;
    else if(firstDir== WEST)  centerPosition.x += 90;
    if(turn_type == Operation::TURN_RIGHT45 || turn_type == Operation::TURN_LEFT45){
      setRobotVecFromRun((turn_type == Operation::TURN_RIGHT45) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
      nextRunVec = RobotRunVec;
      nextDir = directionFromRunVec(RobotRunVec);
    }
    else if(turn_type == Operation::TURN_RIGHT135 || turn_type == Operation::TURN_LEFT135){
      setRobotVecFromRun((turn_type == Operation::TURN_RIGHT135) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
      firstRunVec = RobotRunVec;
      firstDir = directionFromRunVec(RobotRunVec);
      setRobotVecFromRun((turn_type == Operation::TURN_RIGHT135) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
      nextRunVec = RobotRunVec;
      nextDir = directionFromRunVec(RobotRunVec);
    }
    uint16_t target_offset = 0;

    uint16_t target_index = target_offset;
    uint16_t last_index = 0;
    float reference_speed = turn_speed;
    float e_x = 0.0, e_y = 0.0, theta_e = 0.0;
    float last_e_x = 0.0, last_e_theta = 0.0;
    float theta_e_diff = 0.0;
    float last_theta_e = 0.0;
    float theta_e_sum = 0.0;
    float latest_target_x = x(), latest_target_y = y();
    float w_r;

    bool initial_flag = false, second_flag = false;
    slip_rad = 0.0;

    firstDir = directionFromRunVec(operateRunVec);
    Traject traject = trajectList.getTraject(turn_type, firstDir);
    Operation::OperationType nextOn = root[(*i)+1].op;
    float diag_length = 0;
    float dst_len = 0;
    /*
    if(nextOP == Operation::FORWARD_DIAG){
      diag_length = ONE_BLOCK / sqrt(2.0) * root[(*i)+1].n;
      (*i)++;
    }
    */
    while(initial_flag == false){
      if(ENCODER_start == ON){
        read_encoder();
        //slip_rad = ((slip_rad / 0.001) + GYRO_rad) / ((1 / 0.001) + (1 / (slip_k * (left_speed + right_speed) / 2.0 / MmConvWheel)));
        slip_rad = 0.001 * (GYRO_rad - slip_rad * (1.0 / (slip_k * (left_speed + right_speed) / 2.0 / MmConvWheel) - 1.0 / 0.001));
        if(slip_rad > SLIP_MAX) slip_rad = SLIP_MAX;
        else if(slip_rad < -SLIP_MAX) slip_rad = -SLIP_MAX;
        add_coordinate(degree, slip_rad);
        now_speed = now_speed <= reference_speed ? now_speed + accel : reference_speed;
        theta_e_diff = theta_e - last_theta_e;
        theta_e_sum += theta_e / 1000.0;
        float theta_input = (degree_d * theta_e_diff + degree_i * theta_e_sum + degree_p * theta_e);
        //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x + diagKtheta * sin(theta_e)));
        speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin((theta_e - last_e_theta)) + diagKtheta * sin(theta_e)));
        //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x) + theta_input);
        ENCODER_start = OFF;
        start_buzzer(5);
      }
      if(timer_clock == ON){
        timer_clock = OFF;
        Position tmp = getPosition();
        plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
      }
      if(traject_clock == ON){
        traject_clock = OFF;
        last_e_x = e_x;
        last_e_theta = theta_e;
        float update_length = (left_speed + right_speed) / 2 / MmConvWheel * 5.0 / 1000.0;
        dst_len += update_length;
        uint16_t index_size = traject.get_used_size();
        target_index = (target_offset + (uint16_t)dst_len) % index_size;
        dotData dot;
        if(initial_flag == false){
          dot = traject.get_data(target_index, turn_type, firstDir);
        }
        float target_x = dot.x + centerPosition.x;
        float target_y = dot.y + centerPosition.y;
        e_x = target_x - x();
        e_y = target_y - y();
        float tmp = e_x;
        e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
        e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
        w_r = (dot.rad - (traject.get_data((target_index - (uint16_t)update_length) % index_size, turn_type, firstDir).rad)) * 200.0;
        last_theta_e = theta_e;
        theta_e = dot.rad - (degree - target_degree) / 180.0 * PI;//atan2(dango.x() - last_c_x, dango.y() - last_c_y);
        if(last_index > target_index){
          initial_flag = true;
          len_counter = 0;
        }
        else last_index = target_index;
        latest_target_x = target_x;
        latest_target_y = target_y;
      }
    }
    if(turn_type == Operation::TURN_LEFT45) target_degree += 45.0f;
    else if(turn_type == Operation::TURN_RIGHT45) target_degree -= 45.0f;
    else if(turn_type == Operation::TURN_LEFT135) target_degree += 135.0f;
    else if(turn_type == Operation::TURN_RIGHT135) target_degree -= 135.0f;

    float extra_offset = 0.0;
    if(turn_type == Operation::TURN_RIGHT45 || turn_type == Operation::TURN_LEFT45 || turn_type == Operation::TURN_RIGHT135 || turn_type == Operation::TURN_LEFT135){
      len_counter = 0;
      float offset = 5.0;
      float update_length = 0.0;
      Position targetPos;
      if(turn_type == Operation::TURN_RIGHT45 || turn_type == Operation::TURN_LEFT45) extra_offset = 77.27922074784167;
      else extra_offset = 24.55;
      if((firstDir == NORTH && nextDir == WEST) || (firstDir == WEST && nextDir == NORTH)){
        targetPos.x = -1 / sqrt(2.0);
        targetPos.y =  1 / sqrt(2.0);
      }
      else if((firstDir == WEST && nextDir == SOUTH) || (firstDir == SOUTH && nextDir == WEST)){
        targetPos.x = -1 / sqrt(2.0);
        targetPos.y = -1 / sqrt(2.0);
      }
      else if((firstDir == SOUTH && nextDir == EAST) || (firstDir == EAST && nextDir == SOUTH)){
        targetPos.x =  1 / sqrt(2.0);
        targetPos.y = -1 / sqrt(2.0);
      }
      else if((firstDir == EAST && nextDir == NORTH) || (firstDir == NORTH && nextDir == EAST)){
        targetPos.x = 1 / sqrt(2.0);
        targetPos.y = 1 / sqrt(2.0);
      }
      Position startPosition = {
        .x = centerPosition.x + traject.end(turn_type, firstDir).x,
        .y = centerPosition.y + traject.end(turn_type, firstDir).y
      };
      while(len_counter < len_measure(extra_offset)){
        float wall_value = 0.0f;
        float value = 0;
        if(timer_clock == ON){
          timer_clock = OFF;
          Position tmp = getPosition();
          plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
        }
        /*
        if(SENSOR_reset == ON){
          if(led_3 >= led_3_threshold || led_4 >= led_4_threshold ){
            if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)
              wall_value = ((led_4 - led_4_threshold) - (led_3 - led_3_threshold)) / 3.0;
            else if(led_3 >= led_3_threshold)
              wall_value = (-led_3 + led_3_threshold) / 3.0;
            else if(led_4 >= led_4_threshold)
              wall_value = (led_4 - led_4_threshold) / 3.0;
            led_fullon();
            start_buzzer(7);
          }
          else led_fulloff();
          reset_led();
          SENSOR_reset = OFF;
        }
        */
        if(SENSOR_start == ON)	led_get();
        if(ENCODER_start == ON){
          if(len_counter >= len_measure(diag_length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)reference_speed * (int32_t)reference_speed) / 2.0 / (accel * 1000))) //conv to mm
            now_speed = reference_speed >= now_speed ? reference_speed : now_speed - accel;
          else
            now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
          read_encoder();
          add_coordinate(degree);
          float target_theta_now = (degree - target_degree) / 180.0 * PI;
          update_length += (left_speed + right_speed) / 2 / MmConvWheel / 1000.0;
          value = - (degree_p * target_theta_now - sensor_p * wall_value);
          //speed_controller(now_speed, -degree_p * target_theta_now);
          //speed_controller(reference_speed, value);
          latest_target_x = startPosition.x + (update_length + offset) * targetPos.x;
          latest_target_y = startPosition.y + (update_length + offset) * targetPos.y;
          float e_x = latest_target_x - x();
          float e_y = latest_target_y - y();
          float tmp = e_x;
          e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
          e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
          //speed_controller(reference_speed * cos(-target_theta_now) + diagKy * e_y, reference_speed * (-diagKx * e_x + diagKtheta * sin(-target_theta_now)));
          speed_controller(now_speed* cos(-target_theta_now) + diagKy * e_y, now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin(-(target_theta_now - last_e_theta)) + diagKtheta * sin(-target_theta_now)));
          //speed_controller(reference_speed * cos(-target_theta_now) + diagKy * e_y,  reference_speed * (-diagStraightKx * e_x));
          last_e_x = e_x;
          last_e_theta = theta_e;
          ENCODER_start = OFF;
        }
      }
    }
    now_speed = (left_speed + right_speed) / 2 / MmConvWheel;
    (*i)++;

    while(1){
      slip_rad = 0.0;
      second_flag = false;
      Operation::OperationType reverseOP;
      Position startPosition = getPositionFromVec(getRobotVec(), directionFromRunVec(RobotRunVec));
      IndexVec offsetIndex;
      float rad_offset = 0;
      float reverse_offset = 0.0;
      bool reverse_flag = false;
      float target_theta_sum = 0, target_theta_diff = 0.0, target_theta_last = 0.0;
      diag_length = 0;
      Operation::OperationType latestOP = root[*i].op;
      Matrix2i offsetRunVec1st;
      Matrix2i offsetRunVec2nd;
      if(latestOP == Operation::TURN_RIGHT45 || latestOP == Operation::TURN_LEFT45){
        reverse_flag = true;
        firstRunVec = RobotRunVec;
        offsetRunVec1st = RobotRunVec;
        reverseOP = (latestOP == Operation::TURN_RIGHT45) ? Operation::TURN_LEFT45 : Operation::TURN_RIGHT45;
        offsetIndex = RobotVec;
        setRobotVecFromRun((latestOP == Operation::TURN_RIGHT45) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
        nextRunVec = RobotRunVec;
        offsetRunVec2nd = RobotRunVec;
      }
      else if(latestOP == Operation::TURN_RIGHT135 || latestOP == Operation::TURN_LEFT135){
        reverse_flag = true;
        offsetRunVec1st = RobotRunVec;
        reverseOP = (latestOP == Operation::TURN_RIGHT135) ? Operation::TURN_LEFT135 : Operation::TURN_RIGHT135;
        setRobotVecFromRun((latestOP == Operation::TURN_RIGHT135) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
        firstRunVec = RobotRunVec;
        offsetRunVec2nd = RobotRunVec;
        offsetIndex = RobotVec;
        setRobotVecFromRun((latestOP == Operation::TURN_RIGHT135) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
        nextRunVec = RobotRunVec;
      }
      else if (latestOP == Operation::LEFT_V90 || latestOP == Operation::Operation::RIGHT_V90){
        rad_offset = (latestOP == Operation::RIGHT_V90) ? -PI / 4.f : PI / 4.f;
        reverseOP = (latestOP == Operation::LEFT_V90) ? Operation::LEFT_V90 : Operation::RIGHT_V90;
        offsetRunVec1st = RobotRunVec;
        setRobotVecFromRun((latestOP == Operation::RIGHT_V90) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
        firstRunVec = RobotRunVec;
        nextRunVec = RobotRunVec; // fix this at last time
        setRobotVecFromRun((latestOP == Operation::RIGHT_V90) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
      }
      else if(latestOP == Operation::FORWARD_DIAG){
        diag_length = sqrt(2.0) * 90.0 * root[*i].n;
        uint8_t now_count = root[*i].n / 2;
        uint8_t last_count = root[*i].n - now_count;
        IndexVec last_diag_vec(last_count * firstRunVec(0,0), last_count * firstRunVec(0,1));
        IndexVec now_diag_vec(now_count * nextRunVec(0,0), now_count * nextRunVec(0,1));
        addRobotVec(last_diag_vec + now_diag_vec);
        len_counter = 0;
        Direction firstDir = directionFromRunVec(firstRunVec);
        Direction nextDir = directionFromRunVec(nextRunVec);
        Position targetPos;
        float offset = 5.0;
        float update_length = 0.0;
        if((firstDir == NORTH && nextDir == WEST) || (firstDir == WEST && nextDir == NORTH)){
          targetPos.x = -1 / sqrt(2.0);
          targetPos.y =  1 / sqrt(2.0);
        }
        else if((firstDir == WEST && nextDir == SOUTH) || (firstDir == SOUTH && nextDir == WEST)){
          targetPos.x = -1 / sqrt(2.0);
          targetPos.y = -1 / sqrt(2.0);
        }
        else if((firstDir == SOUTH && nextDir == EAST) || (firstDir == EAST && nextDir == SOUTH)){
          targetPos.x =  1 / sqrt(2.0);
          targetPos.y = -1 / sqrt(2.0);
        }
        else if((firstDir == EAST && nextDir == NORTH) || (firstDir == NORTH && nextDir == EAST)){
          targetPos.x = 1 / sqrt(2.0);
          targetPos.y = 1 / sqrt(2.0);
        }
        while(len_counter < len_measure(diag_length)){
          float wall_value = 0.0f;
          if(timer_clock == ON){
            timer_clock = OFF;
            Position tmp = getPosition();
            plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
          }
          if(SENSOR_reset == ON){
            if(led_3 >= led_3_threshold || led_4 >= led_4_threshold ){
              if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)
                wall_value = ((led_4 - led_4_threshold) - (led_3 - led_3_threshold)) / 3.0;
              else if(led_3 >= led_3_threshold)
                wall_value = (-led_3 + led_3_threshold) / 3.0;
              else if(led_4 >= led_4_threshold)
                wall_value = (led_4 - led_4_threshold) / 3.0;
              led_fullon();
              start_buzzer(7);
            }
            else led_fulloff();
            reset_led();
            SENSOR_reset = OFF;
          }
          float value = 0;
          if(SENSOR_start == ON)	led_get();
          if(ENCODER_start == ON){
            if(len_counter >= len_measure(diag_length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)reference_speed * (int32_t)reference_speed) / 2.0 / (accel * 1000))) //conv to mm
              now_speed = reference_speed >= now_speed ? reference_speed : now_speed - accel;
            else
              now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
            read_encoder();
            add_coordinate(degree);
            float target_theta_now = (degree - target_degree) / 180.0 * PI;
            update_length += (left_speed + right_speed) / 2 / MmConvWheel / 1000.0;
            target_theta_sum += target_theta_now / 1000.0;
            float target_theta_diff = target_theta_now - target_theta_last;
            target_theta_last = target_theta_now;
            float theta_input = -(degree_d * target_theta_diff + degree_i * target_theta_sum + degree_p * target_theta_now);
            float value = theta_input + sensor_p * wall_value;
            latest_target_x = startPosition.x + (update_length + offset) * targetPos.x;
            latest_target_y = startPosition.y + (update_length + offset) * targetPos.y;
            float e_x = latest_target_x - x();
            float e_y = latest_target_y - y();
            float tmp = e_x;
            e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
            e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
            //speed_controller(reference_speed * cos(-target_theta_now) + diagKy * e_y, reference_speed * (-diagKx * e_x + diagKtheta * sin(-target_theta_now)));
            //speed_controller(now_speed* cos(-target_theta_now) + diagKy * e_y, now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin((-target_theta_now + last_e_theta)) + diagKtheta * sin(-target_theta_now)));
            speed_controller(now_speed, now_speed * (-(diagStraightKx * e_x + diagKxD * (e_x - last_e_x))) + theta_input);
            last_e_x = e_x;
            last_e_theta = theta_e;
            ENCODER_start = OFF;
          }
        }
        if(root[*i].n % 2 == 1) RobotRunVec = firstRunVec;
      }

      Direction secondDir = directionFromRunVec(nextRunVec);
      Traject secondTraject = trajectList.getTraject(reverseOP, secondDir);
      target_index = target_offset;
      last_index = 0;
      e_x = 0.0f, e_y = 0.0f;
      last_e_x = 0.0, last_e_theta = 0.0;
      theta_e = 0.0f, theta_e_sum = 0.0, last_theta_e = 0.0, theta_e_diff = 0.0, theta_e_diff = 0.0;
      w_r = 0;
      dst_len = 0.0;
      uint16_t index_size = secondTraject.get_used_size();
      dotData lastDot;
      if(reverse_flag == true)
        lastDot = secondTraject.reverse_get_data(0, reverseOP, secondDir);
      else // V90
        lastDot = secondTraject.get_data(0, reverseOP, secondDir);
      
      if(reverse_flag == true){
        if(reverseOP == Operation::TURN_RIGHT45 || reverseOP == Operation::TURN_LEFT45) extra_offset = 77.27922074784167;
        else if(reverseOP == Operation::TURN_RIGHT135 || reverseOP == Operation::TURN_LEFT135) extra_offset = 24.55;//4.55
        len_counter = 0;
        firstDir = directionFromRunVec(offsetRunVec1st);
        nextDir = directionFromRunVec(offsetRunVec2nd);
        Position targetPos;
        float offset = 5.0;
        float update_length = 0.0;
        if((firstDir == NORTH && nextDir == WEST) || (firstDir == WEST && nextDir == NORTH)){
          targetPos.x = -1 / sqrt(2.0);
          targetPos.y =  1 / sqrt(2.0);
        }
        else if((firstDir == WEST && nextDir == SOUTH) || (firstDir == SOUTH && nextDir == WEST)){
          targetPos.x = -1 / sqrt(2.0);
          targetPos.y = -1 / sqrt(2.0);
        }
        else if((firstDir == SOUTH && nextDir == EAST) || (firstDir == EAST && nextDir == SOUTH)){
          targetPos.x =  1 / sqrt(2.0);
          targetPos.y = -1 / sqrt(2.0);
        }
        else if((firstDir == EAST && nextDir == NORTH) || (firstDir == NORTH && nextDir == EAST)){
          targetPos.x = 1 / sqrt(2.0);
          targetPos.y = 1 / sqrt(2.0);
        }
        while(len_counter < len_measure(extra_offset)){
          if(timer_clock == ON){
            timer_clock = OFF;
            Position tmp = getPosition();
            plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
          }
          if(ENCODER_start == ON){
            if(len_counter >= len_measure(diag_length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)reference_speed * (int32_t)reference_speed) / 2.0 / (accel * 1000))) //conv to mm
              now_speed = reference_speed >= now_speed ? reference_speed : now_speed - accel;
            else
              now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
            read_encoder();
            add_coordinate(degree);
            float target_theta_now = (degree - target_degree) / 180.0 * PI;
            update_length += (left_speed + right_speed) / 2 / MmConvWheel / 1000.0;
            latest_target_x = startPosition.x + (update_length + offset) * targetPos.x;
            latest_target_y = startPosition.y + (update_length + offset) * targetPos.y;
            float e_x = latest_target_x - x();
            float e_y = latest_target_y - y();
            float tmp = e_x;
            e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
            e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
            //speed_controller(reference_speed * cos(-target_theta_now) + diagKy * e_y, reference_speed * (-diagKx * e_x + diagKtheta * sin(-target_theta_now)));
            speed_controller(now_speed* cos(-target_theta_now) + diagKy * e_y, now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin(-(target_theta_now - last_e_theta)) + diagKtheta * sin(-target_theta_now)));
            last_e_x = e_x;
            last_e_theta = theta_e;
            ENCODER_start = OFF;
          }
        }
      }
      ////////////////////
      if(latestOP == Operation::RIGHT_V90 || latestOP == Operation::LEFT_V90){
        len_counter = 0;
        while(len_counter < len_measure(30.0)){
          float wall_value = 0.0f;
          if(timer_clock == ON){
            timer_clock = OFF;
            Position tmp = getPosition();
            plot.push_back(tmp.x, tmp.y, 0,0, tmp.vr_x, tmp.vr_y);
          }
          if(SENSOR_reset == ON){
            if(led_3 >= led_3_threshold || led_4 >= led_4_threshold ){
              if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)
                wall_value = ((led_4 - led_4_threshold) - (led_3 - led_3_threshold)) / 4.0;
              else if(led_3 >= led_3_threshold)
                wall_value = (-led_3 + led_3_threshold) / 4.0;
              else if(led_4 >= led_4_threshold)
                wall_value = (led_4 - led_4_threshold) / 4.0;
              led_fullon();
            }
            else led_fulloff();
            reset_led();
            SENSOR_reset = OFF;
          }
          float value = 0;
          if(SENSOR_start == ON)	led_get();
          if(ENCODER_start == ON){
            if(len_counter >= len_measure(diag_length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)reference_speed * (int32_t)reference_speed) / 2.0 / (accel * 1000))) //conv to mm
              now_speed = reference_speed >= now_speed ? reference_speed : now_speed - accel;
            else
              now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
            read_encoder();
            slip_rad = 0.001 * (GYRO_rad - slip_rad * (1.0 / (slip_k * (left_speed + right_speed) / 2.0 / MmConvWheel) - 1.0 / 0.001));
            if(slip_rad > SLIP_MAX) slip_rad = SLIP_MAX;
            else if(slip_rad < -SLIP_MAX) slip_rad = -SLIP_MAX;
            add_coordinate(degree, slip_rad);
            float target_theta_now = (degree - target_degree) / 180.0 * PI;
            value = - (degree_p * target_theta_now - sensor_p * wall_value);
            //speed_controller(now_speed, -degree_p * target_theta_now);
            speed_controller(reference_speed, value);
            ENCODER_start = OFF;
          }
        }
      }
      ////////////////////
      if(reverse_flag == true) startPosition = getPositionFromVec(getRobotVec());
      e_x = 0.0f, e_y = 0.0f;
      last_e_x = 0.0f, last_e_theta;
      theta_e = 0.0f, theta_e_sum = 0.0, last_theta_e = 0.0, theta_e_diff = 0.0, theta_e_diff = 0.0;
      w_r = 0;
      while(second_flag == false && latestOP != Operation::FORWARD_DIAG){
        float wall_value = 0.0f;
        if(SENSOR_reset == ON){
          if(led_3 >= led_3_threshold || led_4 >= led_4_threshold ){
            if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)
              wall_value = ((led_4 - led_4_threshold) - (led_3 - led_3_threshold)) / 6.0;
            else if(led_3 >= led_3_threshold)
              wall_value = (-led_3 + led_3_threshold) / 6.0;
            else if(led_4 >= led_4_threshold)
              wall_value = (led_4 - led_4_threshold) / 6.0;
            led_fullon();
          }
          else led_fulloff();
          reset_led();
          SENSOR_reset = OFF;
        }
        float value = 0;
        if(timer_clock == ON){
          timer_clock = OFF;
          Position tmp = getPosition();
          plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
        }
        if(SENSOR_start == ON)	led_get();
        if(ENCODER_start == ON){
          read_encoder();
          slip_rad = 0.001 * (GYRO_rad - slip_rad * (1.0 / (slip_k * (left_speed + right_speed) / 2.0 / MmConvWheel) - 1.0 / 0.001));
          if(slip_rad > SLIP_MAX) slip_rad = SLIP_MAX;
          else if(slip_rad < -SLIP_MAX) slip_rad = -SLIP_MAX;
          add_coordinate(degree, slip_rad);
          now_speed = now_speed <= reference_speed ? now_speed + accel : reference_speed;
          value = sensor_p * wall_value;
          //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x + diagKtheta * sin(theta_e)) + value);
          //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x + diagKtheta * sin(theta_e)) + value);
          speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin(-(theta_e - last_theta_e)) + diagKtheta * sin(theta_e)));
          theta_e_diff = theta_e - last_theta_e;
          theta_e_sum += theta_e / 1000.0;
          float theta_input = (degree_d * theta_e_diff + degree_i * theta_e_sum + degree_p * theta_e);
          //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x) + theta_input);
          ENCODER_start = OFF;
          start_buzzer(10);
        }
        if(traject_clock == ON){
          traject_clock = OFF;
          last_e_x = e_x;
          last_e_theta = theta_e;
          uint16_t latest_velocity = (left_speed + right_speed) / 2 / MmConvWheel;
          dst_len += latest_velocity * 5.0 / 1000.0;
          target_index = (target_offset + (uint16_t)dst_len) % index_size;
          dotData dot;
          if(reverse_flag == true){
            dot = secondTraject.reverse_get_data(target_index, reverseOP, secondDir);
          }
          else // V90
            dot = secondTraject.get_data(target_index, reverseOP, secondDir);
          float target_x = dot.x + startPosition.x;
          float target_y = dot.y + startPosition.y;
          e_x = target_x - x();
          e_y = target_y - y();
          float tmp = e_x;
          e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
          e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
          w_r = (dot.rad - lastDot.rad) * 200.0;
          last_theta_e = theta_e;
          theta_e = dot.rad + rad_offset - (degree - target_degree) / 180.0 * PI;//atan2(dango.x() - last_c_x, dango.y() - last_c_y);
          if(last_index > target_index){
            second_flag = true;
            len_counter = 0;
          }
          else last_index = target_index;
          latest_target_x = target_x;
          latest_target_y = target_y;
          lastDot = dot;
        }
      }
      if(latestOP == Operation::TURN_RIGHT45 || latestOP == Operation::TURN_LEFT45){
        target_degree += (latestOP == Operation::TURN_RIGHT45) ? -45.0 : 45.0f;
      }
      else if(latestOP == Operation::TURN_RIGHT135 || latestOP == Operation::TURN_LEFT135){
        target_degree += (latestOP == Operation::TURN_RIGHT135) ? -135.0 : 135.0f;
      }
      else if (latestOP == Operation::LEFT_V90 || latestOP == Operation::Operation::RIGHT_V90){
        target_degree += (latestOP == Operation::RIGHT_V90) ? -90.0 : 90.0;
      }
      //////////////
      if(latestOP == Operation::RIGHT_V90 || latestOP == Operation::LEFT_V90){
        len_counter = 0;
        while(len_counter < len_measure(30.0)){
          float wall_value = 0.0f;
          if(timer_clock == ON){
            timer_clock = OFF;
            Position tmp = getPosition();
            plot.push_back(tmp.x, tmp.y, 0,0, tmp.vr_x, tmp.vr_y);
          }
          if(SENSOR_reset == ON){
            if(led_3 >= led_3_threshold || led_4 >= led_4_threshold ){
              if(led_3 >= led_3_threshold && led_4 >= led_4_threshold)
                wall_value = ((led_4 - led_4_threshold) - (led_3 - led_3_threshold)) / 4.0;
              else if(led_3 >= led_3_threshold)
                wall_value = (-led_3 + led_3_threshold) / 4.0;
              else if(led_4 >= led_4_threshold)
                wall_value = (led_4 - led_4_threshold) / 4.0;
              led_fullon();
              //if(led_1 >= led_1_threshold && led_2 >= led_2_threshold && abs(led_1 - led_2) < 150)  fixCoordinate(RobotRunVec, led_1, led_2);
            }
            else led_fulloff();
            reset_led();
            SENSOR_reset = OFF;
          }
          float value = 0;
          if(SENSOR_start == ON)	led_get();
          if(ENCODER_start == ON){
            if(len_counter >= len_measure(diag_length - 1.5 * ((int32_t)now_speed * (int32_t)now_speed - (int32_t)reference_speed * (int32_t)reference_speed) / 2.0 / (accel * 1000))) //conv to mm
              now_speed = reference_speed >= now_speed ? reference_speed : now_speed - accel;
            else
              now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
            read_encoder();
            add_coordinate(degree);
            float target_theta_now = (degree - target_degree) / 180.0 * PI;
            value = - (degree_p * target_theta_now - sensor_p * wall_value);
            //speed_controller(now_speed, -degree_p * target_theta_now);
            speed_controller(reference_speed, value);
            ENCODER_start = OFF;
          }
        }
      }
      //////////////
      if(judge_diag_turn(latestOP)) break;
      nextRunVec = RobotRunVec;
      (*i)++;
    }
  }

  /*
  else if((root[*i].op == Operation::TURN_RIGHT45) || (root[*i].op == Operation::TURN_LEFT45)){
    Operation firstOP = root[*i].op;
    Matrix2i firstRunVec = RobotRunVec;
    IndexVec firstVec = getRobotVec();
    setRobotVecFromRun((root[*i].op == Operation::TURN_RIGHT45) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
    Matrix2i lastRunVec = RobotRunVec;

    float past_target_degree = target_degree;
    float next_target_degree = target_degree;
    int8_t first_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;
    float first_degree_diff = first_diag_direction * 45.0f / 30.0;
    float initial_length = 0.0;
    uint16_t diag_length = 254;
    (*i)++;
    if(root[*i].op == Operation::FORWARD_DIAG){
      diag_length += 127 * root[*i].n;
      uint8_t now_count = root[*i].n / 2;
      uint8_t last_count = root[*i].n - now_count;
      IndexVec last_diag_vec(last_count * firstRunVec(0,0), last_count * firstRunVec(0,1));
      IndexVec now_diag_vec(now_count * lastRunVec(0,0), now_count * lastRunVec(0,1));
      addRobotVec(last_diag_vec + now_diag_vec);
      (*i)++;
    }
    Operation lastOP = root[*i].op;

    int8_t last_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;
    float last_degree_diff = last_diag_direction * 45.0f / 30.0;
    IndexVec lastVec = getRobotVec();

    bool first_flag = false;
    bool last_flag = false;
    float initial_offset = 0;

    //float clothoid_startX = x();
    //float clothoid_startY = y();
    //float sencond_clothoid_startX;
    //float sencond_clothoid_startY;
    //float endX = (firstVec.x + lastVec.x) * 180 - clothoid_startX;
    //float endY = ((firstVec.Y + lastVec.Y) * 180 / 2.0 + 60.0) - clothoid_startY;
    //float *coordinatePointer();
    //if(firstRunVec(0,0) != 0) coordinatePointer = x;
    //else	coordinatePointer = y;

    while(len_counter <= len_measure(diag_length) + initial_offset){
      //while(judgeDiagCoordinate(firstVec,endX,endY,coordinatePointer){
      float distance = 0.0f;
      x_p = 30.0 * 600.0 / now_speed;
      degree_p = 30.0 * 600.0 / now_speed;
      if(ENCODER_start == ON){
        if(fabs(target_degree - past_target_degree) < 45.0 && first_flag == false) target_degree += first_degree_diff;
        else if(first_flag == false){
          first_flag = true;
          initial_length = len_counter;
          target_degree = past_target_degree + first_diag_direction * 45.0;
          next_target_degree = target_degree;
          initial_offset = initial_length;

          //second_clothoid_startX = endX - (x() - clothoid_startX);
          //second_clothoid_startY = endY - (y() - clothoid_starty);
        }

        if(len_counter >= len_measure(diag_length) - initial_length + initial_offset && first_flag == true && last_flag == false) last_flag = true;
        if(last_flag == true)	target_degree += last_degree_diff;
        if(first_flag == true && last_flag == false && len_counter >= len_measure(diag_length - 15.0 * (now_speed * now_speed - turn_speed * turn_speed) / 2.0 / (accel * 1000))) //conv to mm
          now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
        else if(first_flag == true && last_flag == false){
          if(len_counter >= initial_length + 50) now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
          else distance = centerDistance(firstVec,lastVec,firstRunVec, firstOP.op);
        }
        read_encoder();
        add_coordinate(degree);
        target_theta_now = (degree - target_degree) / 180.0 * PI;
        speed_controller(now_speed,- (degree_p * target_theta_now) + distance * x_p);
        target_theta_last = target_theta_now;
        ENCODER_start = OFF;
      }
    }
    target_degree = next_target_degree + last_diag_direction * 45.0;

    if(firstOP.op == lastOP.op)	RobotRunVec = firstRunVec;
    setRobotVecFromRun((root[*i].op == Operation::TURN_RIGHT45) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
  }
  */
  else if((root[*i].op == Operation::TURN_RIGHT90) || (root[*i].op  == Operation::TURN_LEFT90) || (root[*i].op == Operation::TURN_RIGHT180) || root[*i].op == Operation::TURN_LEFT180){
    Operation::OperationType turn_type = root[*i].op;
    Matrix2i firstRunVec = RobotRunVec;
    Matrix2i nextRunVec = RobotRunVec;
    Direction firstDir = directionFromRunVec(RobotRunVec);
    Position centerPosition = getPositionFromVec(getRobotVec(), directionFromRunVec(RobotRunVec));
    if(turn_type == Operation::TURN_RIGHT180 || turn_type == Operation::TURN_LEFT180){
      setRobotVecFromRun((turn_type == Operation::TURN_RIGHT180) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
      setRobotVecFromRun((turn_type == Operation::TURN_RIGHT180) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
    }
    else{
      setRobotVecFromRun(turn_type, root[*i].n);
    }
    if(firstDir == NORTH)  centerPosition.y -= 90;
    else if(firstDir == SOUTH)  centerPosition.y += 90;
    else if(firstDir == EAST)  centerPosition.x -= 90;
    else if(firstDir == WEST)  centerPosition.x += 90;
    uint16_t target_offset = 0;
    uint16_t target_index = target_offset;
    uint16_t last_index = 0;
    float reference_speed = turn_speed;
    float e_x = 0.0, e_y = 0.0, theta_e = 0.0;
    float last_e_x = 0.0, last_e_theta = 0.0;
    float theta_e_diff = 0.0;
    float last_theta_e = 0.0;
    float theta_e_sum = 0.0;
    float latest_target_x = x(), latest_target_y = y();
    float w_r;
    float dst_len;
    float extra_offset = 50.0;
    Traject traject = trajectList.getTraject(turn_type, firstDir);
    bool initial_flag = false;
    slip_rad = 0;
    while(initial_flag == false){
      if(ENCODER_start == ON){
        read_encoder();
        slip_rad = 0.001 * (GYRO_rad - slip_rad * (1.0 / (slip_k * (left_speed + right_speed) / 2.0 / MmConvWheel) - 1.0 / 0.001));
        if(slip_rad > SLIP_MAX) slip_rad = SLIP_MAX;
        else if(slip_rad < -SLIP_MAX) slip_rad = -SLIP_MAX;
        add_coordinate(degree, slip_rad);
        now_speed = now_speed <= reference_speed ? now_speed + accel : reference_speed;
        //speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-diagKx * e_x + diagKtheta * sin(theta_e)));
        speed_controller(now_speed * cos(theta_e) + diagKy * e_y, w_r + now_speed * (-(diagKx * e_x + diagKxD * (e_x - last_e_x)) + diagKthetaD * sin(-(theta_e - last_e_theta)) + diagKtheta * sin(theta_e)));
        ENCODER_start = OFF;
        start_buzzer(5);
      }
      if(timer_clock == ON){
        timer_clock = OFF;
        Position tmp = getPosition();
        plot.push_back(tmp.x, tmp.y, latest_target_x, latest_target_y, tmp.vr_x, tmp.vr_y);
      }
      if(traject_clock == ON){
        traject_clock = OFF;
        last_e_x = e_x;
        last_e_theta = theta_e;
        float update_length = (left_speed + right_speed) / 2 / MmConvWheel * 5.0 / 1000.0;
        dst_len += update_length;
        uint16_t index_size = traject.get_used_size();
        target_index = (target_offset + (uint16_t)dst_len) % index_size;
        dotData dot;
        if(initial_flag == false){
          dot = traject.get_data(target_index, turn_type, firstDir);
        }
        float target_x = dot.x + centerPosition.x;
        float target_y = dot.y + centerPosition.y;
        e_x = target_x - x();
        e_y = target_y - y();
        float tmp = e_x;
        e_x = tmp * cos(degree / 180.0 * PI) + e_y * sin(degree / 180.0 * PI);
        e_y = -tmp * sin(degree / 180.0 * PI) + e_y * cos(degree / 180.0 * PI);
        w_r = (dot.rad - (traject.get_data((target_index - (uint16_t)update_length) % index_size, turn_type, firstDir).rad)) * 200.0;
        last_theta_e = theta_e;
        theta_e = dot.rad - (degree - target_degree) / 180.0 * PI;//atan2(dango.x() - last_c_x, dango.y() - last_c_y);
        if(last_index > target_index){
          initial_flag = true;
          len_counter = 0;
        }
        else last_index = target_index;
        latest_target_x = target_x;
        latest_target_y = target_y;
      }
    }
    if(turn_type == Operation::TURN_LEFT90) target_degree += 90.0f;
    else if(turn_type == Operation::TURN_RIGHT90) target_degree -= 90.0f;
    else if(turn_type == Operation::TURN_LEFT180) target_degree += 180.0f;
    else if(turn_type == Operation::TURN_RIGHT180) target_degree -= 180.0f;
    /*
    int8_t operation_direction = (root[*i].op == Operation::TURN_RIGHT90) ? -1 : 1;
    const float current_degree = degree;
    float target_rad = 0;
    float first_clothoid_degree = 0;
    float second_clothoid_degree = 0;
    bool clothoid_flag = false;

    if(root[(*i)+1].op == root[*i].op){
      target_degree += operation_direction * 180.0; 
      (*i)++;
      setRobotVecFromRun(root[*i].op,root[*i].n);
    }
    else	target_degree += operation_direction * 90.0; 
    uint8_t prescaler = 0;
    while(1){
      if(timer_clock == ON){
        prescaler = (prescaler + 1) % 100;
        timer_clock = OFF;
        //plot.push_back(x(), y(), left_speed / MmConvWheel, right_speed / MmConvWheel, now_speed);
        plot.push_back(x(), y());
        if(prescaler % 2 == 1){
          //plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor(),getRobotVec().x,getRobotVec().y);
          set_left_sensor(led_1);
          set_right_sensor(led_2);
        }
        if(prescaler % 10 == 0){
          stop_buzzer();
          led_fulloff();
        }
      }
      if(SENSOR_reset == ON){
        reset_led();
        SENSOR_reset = OFF;
      }
      if(SENSOR_start == ON)	led_get();
      if(ENCODER_start == ON){
        float radius = 90.0;
        float timing = turn_speed / radius / 0.7; // 0.7 is accel
        read_encoder();
        add_coordinate(degree);
        if(checkZAccel()){
          zStatus = true;
          return;
        }
        if(target_rad <= turn_speed / 90.0 && clothoid_flag == false){
          target_rad += turn_speed / radius / timing;
          first_clothoid_degree = degree;
          clothoid_flag = false;
        }
        else{
          second_clothoid_degree = (clothoid_flag == false) ? target_degree +  5.0 * (current_degree - first_clothoid_degree) : second_clothoid_degree;
          clothoid_flag = true;
          if(operation_direction == -1 && degree <= second_clothoid_degree)	target_rad -= turn_speed / radius / timing;
          if(operation_direction == 1 && degree >= second_clothoid_degree)	target_rad -= turn_speed / radius / timing;
          if(target_rad <= 0)	break;
        }
        speed_controller(turn_speed,(float)operation_direction * target_rad);
        ENCODER_start = OFF;
      }
    }
    GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);
  */
  }
  else if((root[*i].op == Operation::TURN_RIGHT90S) || (root[*i].op  == Operation::TURN_LEFT90S)){
    setRobotVecFromRun(root[*i].op,root[*i].n);
    len_counter = 0;
    sensor_works();
    setWallStatus();
    uint8_t runStatus = 0;
    int8_t operation_direction = (root[*i].op == Operation::TURN_RIGHT90S) ? -1 : 1;
    const float current_degree = degree;
    float target_rad = 0;
    float first_clothoid_degree = 0;
    float second_clothoid_degree = 0;
    bool clothoid_flag = false;

    float last_target_degree = target_degree;
    float slalom_degree_p = 15.0;
    target_degree += operation_direction * 90.0; 
    uint8_t prescaler = 0;
    bool frontThreshold = false;
    sensor_works();
    while(1){
      if(timer_clock == ON){
        prescaler = (prescaler + 1) % 100;
        timer_clock = OFF;
        //plot.push_back(x(), y(), left_speed / MmConvWheel, right_speed / MmConvWheel);
        if(prescaler % 2 == 1){
          //plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor(),getRobotVec().x,getRobotVec().y);
          set_left_sensor(led_1);
          set_right_sensor(led_2);
        }
        if(prescaler % 10 == 0){
          stop_buzzer();
          led_fulloff();
        }
      }

      if(SENSOR_reset == ON){
        if(led_3 >= frontWallThreshold_3 && led_4 >= frontWallThreshold_4) frontThreshold = true; 
        reset_led();
        SENSOR_reset = OFF;
      }
      if(SENSOR_start == ON)	led_get();
      if(ENCODER_start == ON){
        float radius = 50.0;
        float timing = 15.0;
        read_encoder();
        add_coordinate(degree);
        if(checkZAccel()){
          zStatus = true;
          return;
        }
        if(frontWall == true && runStatus % 2 == 0){
          if(frontThreshold == true){
            //fixCoordinate(last_RobotRunVec, 40);
            frontWall = false; // 2320 2640 was
            runStatus++;
            len_counter = 0;
          }
          target_theta_now = (degree - last_target_degree) / 180.0 * PI;
          speed_controller(turn_speed,- (degree_p * target_theta_now));
        }
        else if(frontWall == false && runStatus == 0 && len_counter < len_measure(31)){
          target_theta_now = (degree - last_target_degree) / 180.0 * PI;
          speed_controller(turn_speed,- (slalom_degree_p * target_theta_now));
        }
        else if(frontWall == false && runStatus == 2 && len_counter < len_measure(31)){
          target_theta_now = (degree - target_degree) / 180.0 * PI;
          speed_controller(turn_speed,- (slalom_degree_p * target_theta_now));
        }
        else if(runStatus % 2 == 0){
          len_counter = 0;
          runStatus++;
          led_fullon();
        }
        else if(runStatus == 1 && target_rad <= turn_speed / radius && clothoid_flag == false){
          target_rad += turn_speed / radius / timing;
          first_clothoid_degree = degree;
          speed_controller(turn_speed,(float)operation_direction * target_rad);
          clothoid_flag = false;
        }
        else if(runStatus == 1){
          second_clothoid_degree = (clothoid_flag == false) ? target_degree + (current_degree - first_clothoid_degree) : second_clothoid_degree;
          clothoid_flag = true;
          speed_controller(turn_speed,(float)operation_direction * target_rad);
          if(operation_direction == -1 && degree <= second_clothoid_degree)	target_rad -= turn_speed / radius / timing;
          if(operation_direction == 1 && degree >= second_clothoid_degree)	target_rad -= turn_speed / radius / timing;
          if(target_rad <= 0){
            runStatus = 2;
            len_counter = 0;
          }
        }
        ENCODER_start = OFF;
      }
      if(runStatus == 3)break;
    }
    GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);
  }
  stop_buzzer();
}
void Robot::action(uint8_t value,OperationList runSequence,ParamList parameters){
  led_flash();
  Delay_ms(1000);
  sensor_works();
  set_left_sensor(led_1);
  set_right_sensor(led_2);
  reset_led();
  TIM2->CNT = 0;
  TIM8->CNT = 0;
  degree = 0;
  set_speed(0,0);
  len_counter = 0;
  reset_e();
  pipi(6);
  pipi(5);
  pipi(4);
  pipi(3);

  OperationList reverseRunSequence = reverseOperation(runSequence,0);
  for(size_t i = 0;i < runSequence.size();i++){
    len_counter = 0;
    if(zStatus == true) break;
    robotShortMove(runSequence,parameters[value],&i);
  }
  /*
     if(button_return != 1){
     sensor_works();
     setWallStatus();
     len_counter = 0;
     for(size_t i = 0;i < reverseRunSequence.size();i++){
     len_counter = 0;
     if(zStatus == true) break;
     robotShortMove(reverseRunSequence,parameters[value],&i);
     }
     }
     */
  while(button_return == 1); 
  led_stop();
  zStatus = false;
  set_speed(0,0);
  Delay_ms(1000);
  degree = 0;
  reset_e();
  len_counter = 0;
}
OperationList rebuildOperation(OperationList list){
  OperationList newOPlist;
  for(size_t i = 0;i < list.size();i++){
    Operation::OperationType latestOP = list[i].op;
    if(latestOP == (Operation::TURN_RIGHT90)  || latestOP == Operation::TURN_LEFT90){
      if(latestOP == list[(i) + 1].op){
        if(latestOP == Operation::TURN_RIGHT90)	newOPlist.push_back(Operation::TURN_RIGHT180);
        else newOPlist.push_back(Operation::TURN_LEFT180);
      }
      else newOPlist.push_back(latestOP);
    }
    else newOPlist.push_back(latestOP);
  }
  return newOPlist;
}
OperationList rebuildOperation(OperationList list,bool diagFlag){
  OperationList newOPlist;
  if(diagFlag == false){
    for(size_t i = 0;i < list.size();i++){
      Operation latestOP = list[i];
      Operation pushOP = latestOP;
      if(((latestOP.op == Operation::TURN_RIGHT90) || (latestOP.op == Operation::TURN_LEFT90))){ //For short shushuhu
        Operation lastOP = newOPlist[i-1];
        Operation nextOP = list[i+1];//No problem because lastOP is stop

        if((lastOP.op == Operation::TURN_RIGHT90S) || (lastOP.op == Operation::TURN_LEFT90S))
          pushOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;

        else if(((nextOP.op == Operation::TURN_RIGHT90) || (nextOP.op == Operation::TURN_LEFT90)) && (nextOP.op != latestOP.op))
          pushOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;
        else if(((nextOP.op == Operation::TURN_RIGHT90) || (nextOP.op == Operation::TURN_LEFT90)) && (nextOP.op == latestOP.op)){
          if(i <= list.size() - 3){
            Operation futureOP = list[i+2];//No problem because lastOP is stop
            if((futureOP.op == Operation::TURN_RIGHT90 || futureOP.op == Operation::TURN_LEFT90) && futureOP.op != latestOP.op)
              pushOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;
          }
        }
      }
      if(latestOP.op == Operation::FORWARD){
        Operation nextOP = list[i + 1];
        if(nextOP.op == Operation::FORWARD){
          pushOP.n += nextOP.n;
          i++;
        }
      }
      newOPlist.push_back(pushOP);
      if(((latestOP.op == Operation::TURN_RIGHT90) || (latestOP.op == Operation::TURN_LEFT90))){
        Operation nextOP = list[i+1];//No problem because lastOP is stop
        if(nextOP.op == Operation::STOP)	newOPlist.push_back({Operation::FORWARD,1});
      }
    }
    newOPlist = rebuildOperation(newOPlist, true);
  }
  else{
    for(size_t i = 0;i < list.size();i++){
      Operation latestOP = list[i];
      Operation pushOP = latestOP;
      if((latestOP.op == Operation::TURN_LEFT45 || latestOP.op == Operation::TURN_RIGHT45 || latestOP.op == Operation::TURN_LEFT90 || latestOP.op == Operation::TURN_RIGHT90) && (i+1 < list.size())){
        Operation nextOP = list[i+1];
        if(nextOP.op == latestOP.op && (latestOP.op == Operation::TURN_LEFT45 || latestOP.op == Operation::TURN_RIGHT45)){
          pushOP.op = (latestOP.op == Operation::TURN_LEFT45) ? Operation::LEFT_V90 : Operation::RIGHT_V90;
          i++;
        }
        else if(nextOP.op == Operation::TURN_LEFT90 && latestOP.op == Operation::TURN_LEFT45){
          pushOP.op = Operation::TURN_LEFT135;
          i++;
        }
        else if(nextOP.op == Operation::TURN_LEFT45 && latestOP.op == Operation::TURN_LEFT90){
          pushOP.op = Operation::TURN_LEFT135;
          i++;
        }
        else if(nextOP.op == Operation::TURN_RIGHT90 && latestOP.op == Operation::TURN_RIGHT45){
          pushOP.op = Operation::TURN_RIGHT135;
          i++;
        }
        else if(nextOP.op == Operation::TURN_RIGHT45 && latestOP.op == Operation::TURN_RIGHT90){
          pushOP.op = Operation::TURN_RIGHT135;
          i++;
        }
      }
      if(latestOP.op == Operation::FORWARD){
        Operation nextOP = list[i + 1];
        if(nextOP.op == Operation::FORWARD){
          pushOP.n += nextOP.n;
          i++;
        }
      }
      if((latestOP.op == Operation::TURN_RIGHT90 || latestOP.op == Operation::TURN_LEFT90) && (i+1 < list.size())){
        Operation nextOP = list[i + 1];
        if(latestOP.op == nextOP.op  && latestOP.op == Operation::TURN_RIGHT90){
          pushOP = Operation::TURN_RIGHT180;
          i++;
        }
        else if(latestOP.op == nextOP.op && latestOP.op == Operation::TURN_LEFT90){
          pushOP.op = Operation::TURN_LEFT180;
          i++;
        }
      }
      newOPlist.push_back(pushOP);
      if(((latestOP.op == Operation::TURN_RIGHT90) || (latestOP.op == Operation::TURN_LEFT90))){
        Operation nextOP = list[i+1];//No problem because lastOP is stop
        if(nextOP.op == Operation::STOP)	newOPlist.push_back({Operation::FORWARD,1});
      }
    }
  }
  return newOPlist;
}
Direction directionFromRunVec(Matrix2i runVec){
  if(runVec(0,0) != 0){//about x
    if(runVec(0,0) == 1) return EAST;
    else return WEST;
  }
  else{
    if(runVec(0,1) == 1) return NORTH;
    else return SOUTH;
  }
}

OperationList reverseOperation(OperationList list){
  return reverseOperation(list, 0);
}
OperationList reverseOperation(OperationList list, bool flag){

  OperationList newOPlist;
  for(auto itr = list.rbegin(); itr != list.rend();++itr){
    Operation latestOP = *itr;
    if(latestOP.op == Operation::STOP) continue;
    else if(latestOP.op == Operation::TURN_RIGHT90) latestOP.op = Operation::TURN_LEFT90;
    else if(latestOP.op == Operation::TURN_RIGHT90S) latestOP.op = Operation::TURN_LEFT90S;
    else if(latestOP.op == Operation::TURN_RIGHT180) latestOP.op = Operation::TURN_LEFT180;
    else if(latestOP.op == Operation::TURN_LEFT90) latestOP.op = Operation::TURN_RIGHT90;
    else if(latestOP.op == Operation::TURN_LEFT90S) latestOP.op = Operation::TURN_RIGHT90S;
    else if(latestOP.op == Operation::TURN_LEFT180) latestOP.op = Operation::TURN_RIGHT180;
    newOPlist.push_back(latestOP);
  }
  newOPlist.push_back({Operation::STOP,1});
  return newOPlist;
}
struct Position setStartPosition(Robot &dango){
  IndexVec presentVec = dango.getRobotVec();
  Direction presentDir = directionFromRunVec(dango.RobotRunVec);
  struct Position presentPoint;
  if(presentDir == NORTH && presentDir == SOUTH){
    int8_t which = (presentDir == SOUTH) ? 1 : -1;
    presentPoint.x = ONE_BLOCK * presentVec.x;
    presentPoint.y = ONE_BLOCK * presentVec.y + which * (ONE_BLOCK - WheelFromWall);
  }
  if(presentDir == WEST && presentDir == EAST){
    int8_t which = (presentDir == WEST) ? 1 : -1;
    presentPoint.x = ONE_BLOCK * presentVec.x + which + which * (ONE_BLOCK / 2.f - WheelFromWall);
    presentPoint.y = ONE_BLOCK * presentVec.y + ONE_BLOCK / 2.f - WheelFromWall;
  }
  return presentPoint;
}

float target_Coordinate(IndexVec targetIndex, Matrix2i vecStatus){
  float distance = 0.0f;
  if(vecStatus(0,0) != 0)  //x direction
    distance = (targetIndex.x * 180.0) - vecStatus(0,0) * 90.0;
  else //y direction
    distance = (targetIndex.y * 180.0) - vecStatus(0,1) * 90.0 + 50.0;

  return distance;
}
float Robot::targetLength(IndexVec targetIndex,Matrix2i vecStatus,uint16_t offset){
  if(vecStatus(0,0) != 0){ //x direction
    int8_t direction = vecStatus(0,0);
    return direction * (target_Coordinate(targetIndex,vecStatus) - x() - direction * offset);
  }
  else{
    int8_t direction = vecStatus(0,1);
    return direction * (target_Coordinate(targetIndex,vecStatus) - y() - direction * offset);
  }
}
Position estimatePosition(Position est){
  Position target;
  target.x = ((int32_t)est.x + 90) / 180 * 180.0;
  target.y = ((int32_t)est.y + 50) / 180 * 180.0f + 40.0;
  return target;
}
Position getPositionFromVec(IndexVec vec){
  Position position;
  position.x = vec.x * 180.0;
  position.y = vec.y * 180.0 + 50.0;
  return position;
}
Position getPositionFromVec(IndexVec vec, Direction dir){
  Position position = getPositionFromVec(vec);
  if(dir == NORTH)  position.y -= 90.0;
  else if(dir == SOUTH) position.y += 90.0;
  else if(dir == WEST) position.x += 90.0;
  else if(dir == EAST) position.x -= 90.0;
  return position;
}
bool Robot::judgeTargetCoordinate(IndexVec targetIndex, Matrix2i vecStatus,uint16_t offset){
  return (0 < targetLength(targetIndex,vecStatus,offset)) ? true : false;
}
void Robot::fixCoordinate(){
  Matrix2i vecStatus = RobotRunVec;
  if(vecStatus(0,0) != 0){	//x direction
    uint16_t xCoordinate = (uint8_t)((int16_t)(x() + 90.0) / 180) * 180 ;//+ vecStatus(0,0) * 15.0;
    set_x(xCoordinate);
  }
  else{
    if(y() > 50){
      uint16_t yCoordinate = (uint8_t)((int16_t)(y() + 50.0) / 180) * 180 + 50;// + vecStatus(0,1) * 15.00;
      set_y(yCoordinate);
    }
  }
  led_fullon();
  start_buzzer(2);
}
void Robot::fixCoordinate(Matrix2i runVec, float offset){
  Matrix2i vecStatus = runVec;
  if(vecStatus(0,0) != 0){	//x direction
    //uint16_t xCoordinate = (uint16_t)((int16_t)(x() + 90.0) / 180) * 180 - vecStatus(0,0) * (90 - offset);
    uint16_t xCoordinate = (uint16_t)((int16_t)(x() + 90.0) / 180) * 180 - vecStatus(0,0) * offset;;
    set_x(xCoordinate);
  }
  else{
    if(y() > 40){
      //uint16_t yCoordinate = (uint16_t)((int16_t)(y() + 50.0) / 180) * 180 + 40 - vecStatus(0,1) * (90 - offset);
      uint16_t yCoordinate = (uint16_t)((int16_t)(y() + 45.0) / 180) * 180 + 40 - vecStatus(0,1) * offset;
      set_y(yCoordinate);
    }
  }
  led_fullon();
  start_buzzer(4);
}
void Robot::fixCoordinate(Matrix2i runVec, float wall_left, float wall_right){
  if(wall_left < led_1_threshold && wall_right < led_2_threshold) return;
  Matrix2i vecStatus = runVec;
  const float leftA = -7.0;
  const float rightA = -6.0;
  float A, sub;
  const float b = 7.5714;
  if(wall_left > led_1_threshold && wall_right > led_2_threshold && fabs(wall_left - wall_right) < 250){
    A = -11.232;
    sub = wall_left - wall_right;
  }
  else return;
  /*
  else if(wall_left > led_1_threshold + 200 && wall_right < led_2_threshold){
    A = leftA;
    sub = wall_left - led_1_reference;
  }
  else if(wall_left < led_1_threshold && wall_right > led_2_threshold + 200){
    A = rightA;
    sub = led_2_reference - wall_right;
  }*/
  /*
  const float A = -11.2321;
  const float b = 7.5714;
  */
  float position = (sub - b)  / A;
  if(vecStatus(0,0) != 0){	//x direction fix y
    float yCoordinate = (uint16_t)((int16_t)(y() + 50.0) / 180) * 180 + 50 - vecStatus(0,0) * position;
    set_y(yCoordinate);
  }
  else{//y direction fix x
    float xCoordinate = (uint16_t)((int16_t)(x() + 90.0) / 180) * 180 + vecStatus(0,1) * position;
    set_x(xCoordinate);
  }
  led_fullon();
  start_buzzer(4);
}

float Robot::centerDistance(IndexVec firstVec,IndexVec lastVec,Matrix2i vecStatus, Operation::OperationType op){
  float startX,startY;
  Matrix2f targetPoint;
  if(vecStatus(0,0) != 0){
    startX = target_Coordinate(firstVec,vecStatus);
    startY = firstVec.y * 180.0 + 40.0;
    targetPoint << y() - startY,x() - startX,0,0;
  }
  else{
    startY = target_Coordinate(firstVec,vecStatus);
    startX = firstVec.x * 180.0 - 90.0;
    targetPoint << x() - startX,y() - startY,0,0;
  }
  Matrix2f rotation;
  int8_t turn_dir;
  if(op == Operation::TURN_RIGHT45) turn_dir = 45;
  else turn_dir = -45;
  rotation << cos(turn_dir),-sin(turn_dir),sin(turn_dir),cos(turn_dir);
  targetPoint = targetPoint * rotation;

  if(vecStatus(0,0) != 0)	return  -vecStatus(0,0) * targetPoint(0,1); //50.0
  else	return vecStatus(0,1) * targetPoint(0,0);
}

bool judgeDiagCoordinate(Matrix2i firstVec,float targetX,float targetY,float coordinatePoint){
  if(firstVec(0,0) != 0)	return  (0 < firstVec(0,0) *	(targetX - coordinatePoint)) ? true : false;
  else	return  (0 < firstVec(0,1) *	(targetY - coordinatePoint)) ? true : false;
}
Matrix2i getRotate(Operation::OperationType type){
  if(type == Operation::TURN_RIGHT90 || type == Operation::TURN_RIGHT45) return right90Rotate();
  if(type == Operation::FORWARD) return eigenRotate();
  if(type == Operation::TURN_LEFT90 || type == Operation::TURN_LEFT45) return left90Rotate();
  else return eigenRotate();
}
bool judge_diag_turn(Operation::OperationType op){
  if(op == Operation::TURN_LEFT135 || op == Operation::TURN_LEFT45 || op == Operation::TURN_RIGHT45 || op == Operation::TURN_RIGHT135)  return true;
  else return false;
}
