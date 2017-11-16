#include "mine.h"
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
}
void Robot::add_coordinate(float degree){
	float curent_speed = (left_speed + right_speed) / 2.0 / MmConvWheel / 1000.0;
	float coordinate_x = -sin(degree * PI /180.0);
	float coordinate_y = cos(degree * PI / 180.0);
	add_x(curent_speed * coordinate_x);
	add_y(curent_speed * coordinate_y);
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
	}
		sensor_sub /= 10;
	agent->update(Robot::getRobotVec(),0b11111110);
	while(len_counter <= len_measure(150)){
		//go_straight(0.01 * degree);
		go_straight(0);
	}
	Robot::addRobotVec(IndexVec::vecNorth);
	len_counter = 0;
	//set_speed(0,0);
}
float Robot::runningCoordinate(){
	if(RobotRunVec(0,0) != 0) return x();
	else return y();
}

void Robot::setRobotVecFromRun(uint8_t dir, uint8_t n){
	Matrix2i nextDir;
	if((dir != Operation::TURN_RIGHT45) && (dir != Operation::TURN_LEFT45)){
		if(dir == Operation::FORWARD)
			nextDir = getFORWARD();
		else if((dir == Operation::TURN_RIGHT90) || (dir == Operation::TURN_RIGHT90S))
			nextDir = getRIGHT();
		else if((dir == Operation::TURN_LEFT90) || (dir == Operation::TURN_LEFT90S))
			nextDir = getLEFT();
		RobotRunVec = RobotRunVec * nextDir;
		IndexVec NextState(n * RobotRunVec(0,0),n * RobotRunVec(0,1));
		RobotVec += NextState;
	}
}
void Robot::goStraight(){
	while(len_counter <= len_measure(ONE_BLOCK)){
		float wall_value = 0.0f;
		if(SENSOR_reset == ON){
			if(sideWall == true && led_1 >= led_1_threshold && led_2 >= led_2_threshold) wall_value = (led_2 - led_1 - sensor_sub) / 15.0;
			reset_led();
			SENSOR_reset = OFF;
		}
		if(SENSOR_start == ON)	led_get();
		float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI - wall_value;
		go_straight(target_theta);
	}
}
void Robot::goStraight(uint16_t length){
	while(len_counter <= len_measure(length)){
		float wall_value = 0.0f;
		if(SENSOR_reset == ON){
			if(sideWall == true && led_1 >= led_1_threshold && led_2 >= led_2_threshold) wall_value = (led_2 - led_1 - sensor_sub) / 15.0;
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
				if(led_3 >= 2440 && led_4 >= 2750) frontThreshold = true; // 2320 2640 was
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
	uint16_t offset = 40;
	len_counter = 0;
	slalomStraight(offset);
	len_counter = 0;
	addRobotDegreeDir(1);
	go_left(getRobotDegreeDir() * 90);
	len_counter = 0;
	goStraight(offset);
}
void Robot::startBack(){
	setWallStatus();
	len_counter = 0;
	goStraight(70);
	int8_t wall_dir = 0; // 1 right -1 left
	if(leftWall == true) wall_dir = -1;
	else if(rightWall == true) wall_dir = 1;
	////////////////////////////////////////write here!!!!!!!!!!!!!!!!!!!!!!!!
	len_counter = 0;
	set_speed(0,0);
	Delay_ms(300);
	if(wall_dir != 0){
		turn_side(getRobotDegreeDir(),wall_dir);
		addRobotDegreeDir(wall_dir);
		while(len_counter > len_measure(-70)){
			const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
			go_back(-target_theta);
		}
		degree = getRobotDegreeDir() * 90.0 ;
		reset_e();
		len_counter = 0;
		set_speed(0,0);
		Delay_ms(100);
		while(len_counter < len_measure(30)){
			const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
			go_straight(target_theta);
		}
		set_speed(0,0);
		Delay_ms(100);
		turn_side(getRobotDegreeDir(),wall_dir);
		addRobotDegreeDir(wall_dir);
	}
	else turn_back(getRobotDegreeDir());
	len_counter = 0;
	while(len_counter > len_measure(-75)){
		const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
		go_back(-target_theta);
	}
	reset_e();
	degree = 0;
	set_speed(0,0);
	len_counter = 0;
	setRobotDegreeDir(NORTH);
}
void Robot::goBack(int8_t Nextdir){
	goStraight(70);
	int8_t wall_dir = 0; // 1 right -1 left
	if(leftWall == true) wall_dir = -1;
	else if(rightWall == true) wall_dir = 1;
	////////////////////////////////////////write here!!!!!!!!!!!!!!!!!!!!!!!!
	len_counter = 0;
	set_speed(0,0);
	Delay_ms(300);
	int8_t value;
	if(Nextdir == NORTH)	value = 0;
	if(Nextdir == WEST)	value = 1;
	if(Nextdir == EAST)	value = -1;
	if(Nextdir == SOUTH)	value = 2;
	if(wall_dir != 0){
		turn_side(getRobotDegreeDir(),wall_dir);
		addRobotDegreeDir(wall_dir);
		while(len_counter > len_measure(-70)){
			const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
			go_back(-target_theta);
		}
		if(frontWall == false)	setRobotDegreeDir(value - wall_dir);
		degree = getRobotDegreeDir() * 90.0 ;
		reset_e();
		len_counter = 0;
		set_speed(0,0);
		Delay_ms(100);
		while(len_counter < len_measure(30)){
			const float target_theta =  (degree - getRobotDegreeDir() * 90.0) / 180.0 * PI;
			go_straight(target_theta);
		}
		set_speed(0,0);
		Delay_ms(100);
		turn_side(getRobotDegreeDir(),wall_dir);
		addRobotDegreeDir(wall_dir);
	}
	else turn_back(getRobotDegreeDir());
	if(wall_dir != 0 && frontWall == true)	setRobotDegreeDir(value);
	if(frontWall == false)	start_withoutwall(getRobotDegreeDir());
	else start_wall(getRobotDegreeDir());
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
void Robot::robotMove(Direction Nextdir){
	setWallStatus();
	len_counter = 0;
	Direction Nowdir = Robot::getRobotDir();
	if(Nowdir == NORTH){
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
	if(Nowdir == WEST){
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
	if(Nowdir == SOUTH){
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
	if(Nowdir == EAST){
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
static float target_degree = 0.0f;
static int16_t now_speed = 0;
void Robot::robotShortMove(OperationList root,Param param,size_t *i){
	if(*i == 0){
		target_degree = 0.0f;
		now_speed = (left_speed + right_speed) / 2 / MmConvWheel;
	}
	len_counter = 0;
	uint16_t curving_length = param.get_turn_param() / 50; // 60
	if(root[(*i)].op != Operation::STOP){
		if(root[(*i)+1].op == Operation::STOP) curving_length += 90;
	}

	const int16_t last_speed = param.get_last_param();
	uint16_t turn_speed;
	if(root[(*i)+1].op == Operation::TURN_RIGHT90S || root[(*i)+1].op == Operation::TURN_LEFT90S)	turn_speed = param.get_small_turn_param();
	else if(root[(*i)].op == Operation::TURN_RIGHT90S || root[(*i)].op == Operation::TURN_LEFT90S)	turn_speed = param.get_small_turn_param();
	else turn_speed = param.get_turn_param();
	const int16_t accel = param.get_accel_param();
	uint16_t length = 0;
	curving_length = (root[(*i)+1].op == Operation::TURN_RIGHT45 || root[(*i)+1].op == Operation::TURN_LEFT45) ? curving_length + 45 : curving_length;
	length = *i == 0 ? 150 + (root[*i].n - 1) * ONE_BLOCK - curving_length : root[*i].n * ONE_BLOCK - curving_length; //130 was

	float e_now = 0,e_sum = 0;
	float target_theta_now = 0,target_theta_last = 0;
	float x_p = 0.1 * 600.0 / last_speed;
	float x_i = 0.1 * 600.0 / last_speed;
	float degree_p = 20.0 * 600.0 / last_speed;
	float degree_d = 1.0 * 600.0 / last_speed;
	float sensor_p = 0.5 * 600.0 / last_speed;

	setRobotVecFromRun(root[*i].op,root[*i].n);
	if(root[*i].op == Operation::FORWARD){
		sensor_works();
		set_left_sensor(led_1);
		set_right_sensor(led_2);
		reset_led();
		uint8_t prescaler = 0;

		while(judgeTargetCoordinate(getRobotVec(),RobotRunVec,curving_length)){
			float wall_value = 0.0f;
			if(now_speed >= 1.0){
				x_p = 0.1 * 600.0 / now_speed;
				//x_i = 0.05 * 600.0 / now_speed;
				degree_p = 20.0 * 600.0 / now_speed;
				degree_d = 0.0 * 600.0 / now_speed;
				sensor_p = 0.5 * 600.0 / last_speed;
			}
			if(timer_clock == ON){
				prescaler = (prescaler + 1) % 100;
				plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor(),getRobotVec().x,getRobotVec().y);
				//plot.push_back(now_speed,(left_speed + right_speed) / 2 / MmConvWheel,x(),y());

				timer_clock = OFF;
				if(prescaler % 2 == 0){
					plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor());
					if(targetLength(getRobotVec(),RobotRunVec,curving_length) <= 100 && (fabs(led_1 - get_left_sensor()) > 120 || fabs(led_2 - get_right_sensor()) > 120))	fixCoordinate();
					set_left_sensor(led_1);
					set_right_sensor(led_2);
				}
				if(prescaler % 10 == 0){
					stop_buzzer();
					led_fulloff();
				}
			}
			if(SENSOR_reset == ON){
				if(led_1 >= led_1_threshold && led_2 >= led_2_threshold) wall_value = (led_2 - led_1 - sensor_sub) / 20.0;
				reset_led();
				SENSOR_reset = OFF;
			}
			if(SENSOR_start == ON)	led_get();

			if(ENCODER_start == ON){
				if(checkZAccel()){
					zStatus = true;
					return;
				}
				if(len_counter >= len_measure(length - 5.0 * (now_speed * now_speed - turn_speed * turn_speed) / 2.0 / (accel * 1000))) //conv to mm
					now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
				else
					now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
				read_encoder();
				add_coordinate(degree);
				e_now =  centerDistance();
				e_sum += e_now / 1000.0f;
				target_theta_now = (degree - target_degree) / 180.0 * PI;
				speed_controller(now_speed,- (degree_p * target_theta_now - sensor_p * wall_value)/*+ (-target_theta_now + target_theta_last) * degree_d) + e_now * x_p + e_sum * x_i*/ );
				target_theta_last = target_theta_now;
				ENCODER_start = OFF;
			}
		}
	}
	else if(root[*i].op == Operation::STOP){
		set_speed(0,0);
	}
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
				plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor());
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

	else if((root[*i].op == Operation::TURN_RIGHT90) || (root[*i].op  == Operation::TURN_LEFT90)){
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
				if(prescaler % 2 == 1){
					plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor(),getRobotVec().x,getRobotVec().y);
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
				read_encoder();
				add_coordinate(degree);
				if(checkZAccel()){
					zStatus = true;
					return;
				}
				if(target_rad <= turn_speed / 90.0 && clothoid_flag == false){
					target_rad += turn_speed / 90.0 / 10.0;
					first_clothoid_degree = degree;
					speed_controller(turn_speed,(float)operation_direction * target_rad);
					clothoid_flag = false;
				}
				else{
					second_clothoid_degree = (clothoid_flag == false) ? target_degree + (current_degree - first_clothoid_degree) : second_clothoid_degree;
					clothoid_flag = true;
					speed_controller(turn_speed,(float)operation_direction * target_rad);
					if(operation_direction == -1 && degree <= second_clothoid_degree)	target_rad -= turn_speed / 90.0 / 10.0;
					if(operation_direction == 1 && degree >= second_clothoid_degree)	target_rad -= turn_speed / 90.0 / 10.0;
					if(target_rad <= 0)	break;
				}
				ENCODER_start = OFF;
			}
		}
		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);
	}
	else if((root[*i].op == Operation::TURN_RIGHT90S) || (root[*i].op  == Operation::TURN_LEFT90S)){
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
		target_degree += operation_direction * 90.0; 
		uint8_t prescaler = 0;
		while(1){
			if(timer_clock == ON){
				prescaler = (prescaler + 1) % 100;
				timer_clock = OFF;
				if(prescaler % 2 == 1){
					plot.push_back(x(),y(),degree,get_left_sensor(),get_right_sensor(),getRobotVec().x,getRobotVec().y);
					set_left_sensor(led_1);
					set_right_sensor(led_2);
				}
				if(prescaler % 10 == 0){
					stop_buzzer();
					led_fulloff();
				}
			}
			if(SENSOR_start == ON)	led_get();
			degree_p = 15.0 * 600.0 / now_speed;
			if(ENCODER_start == ON){
				read_encoder();
				add_coordinate(degree);
				if(checkZAccel()){
					zStatus = true;
					return;
				}
				if(frontWall == true && runStatus % 2 == 0){
					if(led_3 >= 2450 && led_4 >= 2760){
						frontWall = false; // 2320 2640 was
						runStatus++;
						len_counter = 0;
					}
					target_theta_now = (degree - last_target_degree) / 180.0 * PI;
					speed_controller(turn_speed,- (degree_p * target_theta_now));
				}
				else if(frontWall == false && runStatus == 0 && len_counter < len_measure(40)){
					target_theta_now = (degree - last_target_degree) / 180.0 * PI;
					speed_controller(turn_speed,- (degree_p * target_theta_now));
				}
				else if(frontWall == false && runStatus == 2 && len_counter < len_measure(40)){
					target_theta_now = (degree - target_degree) / 180.0 * PI;
					speed_controller(turn_speed,- (degree_p * target_theta_now));
				}
				else if(runStatus % 2 == 0){
					len_counter = 0;
					runStatus++;
					led_fullon();
				}
				else if(runStatus == 1 && target_rad <= turn_speed / 40.0 && clothoid_flag == false){
					target_rad += turn_speed / 40.0 / 40.0;
					first_clothoid_degree = degree;
					speed_controller(turn_speed,(float)operation_direction * target_rad);
					clothoid_flag = false;
				}
				else if(runStatus == 1){
					second_clothoid_degree = (clothoid_flag == false) ? target_degree + (current_degree - first_clothoid_degree) : second_clothoid_degree;
					clothoid_flag = true;
					speed_controller(turn_speed,(float)operation_direction * target_rad);
					if(operation_direction == -1 && degree <= second_clothoid_degree)	target_rad -= turn_speed / 40.0 / 40.0;
					if(operation_direction == 1 && degree >= second_clothoid_degree)	target_rad -= turn_speed / 40.0 / 40.0;
					if(target_rad <= 0){
						runStatus = 2;
						len_counter = 0;
					}
				}
				ENCODER_start = OFF;
			}
			if(SENSOR_reset == ON){
				reset_led();
				SENSOR_reset = OFF;
			}
			if(runStatus == 3)break;
		}
		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);
	}
	stop_buzzer();
}
void Robot::action(uint8_t value,OperationList runSequence,ParamList parameters){
	led_flash();
	Delay_ms(3000);
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

	for(size_t i = 0;i<runSequence.size();i++){
		len_counter = 0;
		if(zStatus == true) break;
		robotShortMove(runSequence,parameters[value],&i);
	}
	while(button_return == 1); 
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
		if(latestOP == (Operation::TURN_RIGHT90 || Operation::TURN_LEFT90)){
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
	for(size_t i = 0;i < list.size();i++){
		Operation latestOP = list[i];
		if(((latestOP.op == Operation::TURN_RIGHT90) || (latestOP.op == Operation::TURN_LEFT90))){
			Operation lastOP = newOPlist[i-1];
			Operation nextOP = list[i+1];//No problem because lastOP is stop

			if((lastOP.op == Operation::TURN_RIGHT90S) || (lastOP.op == Operation::TURN_LEFT90S))
				latestOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;
			
			else if(((nextOP.op == Operation::TURN_RIGHT90) || (nextOP.op == Operation::TURN_LEFT90)) && (nextOP.op != latestOP.op))
				latestOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;
			else if(((nextOP.op == Operation::TURN_RIGHT90) || (nextOP.op == Operation::TURN_LEFT90)) && (nextOP.op == latestOP.op)){
				if(i <= list.size() - 3){
					Operation futureOP = list[i+2];//No problem because lastOP is stop
					if((futureOP.op == Operation::TURN_RIGHT90 || futureOP.op == Operation::TURN_LEFT90) && futureOP.op != latestOP.op)
						latestOP.op = (latestOP.op == Operation::TURN_LEFT90) ? Operation::TURN_LEFT90S : Operation::TURN_RIGHT90S;
				}
			}
		}
		newOPlist.push_back(latestOP);
	}
	return newOPlist;
}

float target_Coordinate(IndexVec targetIndex, Matrix2i vecStatus){
	float distance = 0.0f;
	if(vecStatus(0,0) != 0)  //x direction
		distance = (targetIndex.x * 180.0) - vecStatus(0,0) * 90.0;
	else //y direction
		distance = (targetIndex.y * 180.0) - vecStatus(0,1) * 90.0 + 60.0;

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

bool Robot::judgeTargetCoordinate(IndexVec targetIndex, Matrix2i vecStatus,uint16_t offset){
	return (0 < targetLength(targetIndex,vecStatus,offset)) ? true : false;
}
void Robot::fixCoordinate(){
	Matrix2i vecStatus = RobotRunVec;
	if(vecStatus(0,0) != 0){	//x direction
		uint16_t xCoordinate = (uint8_t)((int16_t)(x() + 90.0) / 180) * 180 + vecStatus(0,0) * 15.0;
		set_x(xCoordinate);
	}
	else{
		if(y() > 60){
			uint16_t yCoordinate = (uint8_t)((int16_t)(y() + 30.0) / 180) * 180 + 60 + vecStatus(0,1) * 15.00;
			set_y(yCoordinate);
		}
	}
	led_fullon();
	start_buzzer(2);
}
float Robot::centerDistance(IndexVec firstVec,IndexVec lastVec,Matrix2i vecStatus, Operation::OperationType op){
	float startX,startY;
	Matrix2f targetPoint;
	if(vecStatus(0,0) != 0){
		startX = target_Coordinate(firstVec,vecStatus);
		startY = firstVec.y * 180.0 + 60.0;
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
