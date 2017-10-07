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
	if(dir==NORTH)
		addRobotVec(IndexVec::vecNorth);
	else if(dir==WEST)
		addRobotVec(IndexVec::vecWest);
	else if(dir==SOUTH)
		addRobotVec(IndexVec::vecSouth);
	else if(dir==EAST)
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
void Robot::add_coordinate(float coordinate_x, float coordinate_y){
	float speed = (left_speed + right_speed) / 2.0 / MmConvWheel / 1000.0;
	add_x(speed * coordinate_x);
	add_y(speed * coordinate_y);
}
float Robot::centerDistance(){
	float distance = 0.0f;
	if(RobotRunVec(0,0) != 0)
		distance = RobotRunVec(0,0) * (RobotVec.y * 180.0  + 50.0 - y());
	else
		distance = - RobotRunVec(0,1) * (RobotVec.x * 180.0 - x());
	return distance;
}
void Robot::startOffSet(Agent *agent){
	sensor_works();
	sensor_sub = led_1 - led_2;
	Direction WallData = read_wall(NORTH);
	agent->update(Robot::getRobotVec(),WallData);
	while(len_counter <= len_measure(150)){
		//go_straight(0.01 * degree);
		go_straight(0);
	}
	Robot::addRobotVec(IndexVec::vecNorth);
	len_counter = 0;
	//set_speed(0,0);
}
void Robot::setRobotVecFromRun(uint8_t Direction, uint8_t n){
	Matrix2i nextDir;
	if((Direction != Operation::TURN_RIGHT45) && (Direction != Operation::TURN_LEFT45)){
		if(Direction == Operation::FORWARD)
			nextDir = getFORWARD();
		else if(Direction == Operation::TURN_RIGHT90)
			nextDir = getRIGHT();
		else if(Direction == Operation::TURN_LEFT90)
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
			if(sideWall == true && led_1 >= 3120 && led_2 >= 2970) wall_value = (led_1 - led_2 - sensor_sub) / 25.0;
			reset_led();
			SENSOR_reset = OFF;
		}
		if(SENSOR_start == ON)	led_get();
		float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI + wall_value;
		go_straight(target_theta);
	}
}
void Robot::goStraight(uint16_t length){
	while(len_counter <= len_measure(length)){
		float wall_value = 0.0f;
		if(SENSOR_reset == ON){
			if(sideWall == true && led_1 >= 3120 && led_2 >= 2970) wall_value = (led_1 - led_2 - sensor_sub) / 25.0;
			reset_led();
			SENSOR_reset = OFF;
		}
		if(SENSOR_start == ON)	led_get();
		float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI + wall_value;
		go_straight(target_theta);
	}
}
void Robot::goRight(){
	uint16_t offset = 10;
	goStraight(offset);
	len_counter = 0;
	addRobotDegreeDir(-1);
	go_right(getRobotDegreeDir() * 90 );
	len_counter = 0;
	goStraight(offset);
	//goStraight(ONE_BLOCK / 2 - turn_R);
}
void Robot::goLeft(){
	uint16_t offset = 10;
	goStraight(offset);
	len_counter = 0;
	addRobotDegreeDir(1);
	go_left(getRobotDegreeDir() * 90);
	len_counter = 0;
	goStraight(offset);
}
void Robot::goBack(int8_t Nextdir){
	turn_back(getRobotDegreeDir());
	int8_t value;
	if(Nextdir == NORTH)	value = 0;
	if(Nextdir == WEST)	value = 1;
	if(Nextdir == EAST)	value = -1;
	if(Nextdir == SOUTH)	value = 2;
	setRobotDegreeDir(value);
	start_wall(getRobotDegreeDir());
}
void Robot::setSideWall(){
	if(led_1 >= 3120 && led_2 >= 2970)	sideWall = true;
	else sideWall = false;
}
void Robot::robotMove(Direction Nextdir){
	setSideWall();
	len_counter = 0;
	Direction Nowdir =  Robot::getRobotDir();
	if(Nowdir==NORTH){
		if(Nextdir==NORTH){
			goStraight();
		}
		if(Nextdir==EAST){
			goRight();
		}
		if(Nextdir==WEST){
			goLeft();
		}
		if(Nextdir==SOUTH){
			goBack(Nextdir);
		}
	}
	if(Nowdir==WEST){
		if(Nextdir==NORTH){
			goRight();
		}
		if(Nextdir==EAST){
			goBack(Nextdir);
		}
		if(Nextdir==WEST){
			goStraight();
		}
		if(Nextdir==SOUTH){
			goLeft();
		}
	}
	if(Nowdir==SOUTH){
		if(Nextdir==NORTH){
			goBack(Nextdir);
		}
		if(Nextdir==EAST){
			goLeft();
		}
		if(Nextdir==WEST){
			goRight();
		}
		if(Nextdir==SOUTH){
			goStraight();
		}
	}
	if(Nowdir==EAST){
		if(Nextdir==NORTH){
			goLeft();
		}
		if(Nextdir==EAST){
			goStraight();
		}
		if(Nextdir==WEST){
			goBack(Nextdir);
		}
		if(Nextdir==SOUTH){
			goRight();
		}
	}
}
	void Robot::robotShortMove(OperationList root,Param param,size_t *i){
		//USART_printf("x-- %d y--%d\r\n",RobotVec.x,RobotVec.y);
		//USART_printf("Run %d %d %d %d\r\n",RobotRunVec(0,0),RobotRunVec(0,1),RobotRunVec(1,0),RobotRunVec(1,1));
		len_counter = 0;
		static float target_degree = 0.0f;

		const uint16_t curving_length = param.get_turn_param() / 20; // 60
		static int16_t now_speed = (left_speed + right_speed) / 2 / MmConvWheel;
		const int16_t last_speed = param.get_last_param();
		const int16_t turn_speed = param.get_turn_param();
		const int16_t accel = param.get_accel_param();
		uint16_t length = 0;
		length = *i == 0 ? 150 + (root[*i].n - 1) * ONE_BLOCK - curving_length : root[*i].n * ONE_BLOCK - curving_length; //130 was

		float e_now = 0,e_sum = 0;
		float target_theta_now = 0,target_theta_last = 0;
		float x_p = 0.1 * 600.0 / last_speed;
		float degree_p = 50.0 * 600.0 / last_speed;
		float degree_d = 1.0 * 600.0 / last_speed;

		setRobotVecFromRun(root[*i].op,root[*i].n);
		if(root[*i].op == Operation::FORWARD){
			while(len_counter <= len_measure(length)){
				if(now_speed >= 1.0){
					x_p = 0.15 * 600.0 / now_speed;
					degree_p = 50.0 * 600.0 / now_speed;
					degree_d = 1.0 * 600.0 / now_speed;
				}
				if(ENCODER_start == ON){
					if(len_counter >= len_measure(length - 5.0 * (now_speed * now_speed - turn_speed * turn_speed) / 2.0 / (accel * 1000))) //conv to mm
						now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
					else
						now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
					read_encoder();
					add_coordinate(-sin(degree * PI /180.0),cos(degree * PI / 180.0));
					e_now =  centerDistance();
					e_sum += e_now / 1000.0f;
					target_theta_now = (degree - target_degree * 1.03) / 180.0 * PI;
					speed_controller(now_speed,- (degree_p * target_theta_now + (-target_theta_now + target_theta_last) * degree_d) + e_now * x_p);
					target_theta_last = target_theta_now;
					ENCODER_start = OFF;
				}
			}
		}
		else if(root[*i].op == Operation::STOP){
			set_speed(0,0);
		}
		else if((root[*i].op == Operation::TURN_RIGHT45) || (root[*i].op == Operation::TURN_LEFT45)){
			Matrix2i firstRunVec = RobotRunVec;
			setRobotVecFromRun((root[*i].op == Operation::TURN_RIGHT45) ? Operation::TURN_RIGHT90 : Operation::TURN_LEFT90,root[*i].n);
			Matrix2i lastRunVec = RobotRunVec;

			float past_target_degree = target_degree;
			float next_target_degree = target_degree;
			int8_t first_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;
			float first_degree_diff = first_diag_direction * 45.0f / 100.0;
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
			int8_t last_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;
			float last_degree_diff = last_diag_direction * 45.0f / 100.0;
			bool first_flag = false;
			bool last_flag = false;
			while(len_counter <= len_measure(diag_length)){
				if(ENCODER_start == ON){
					if(fabs(target_degree - past_target_degree) < 45.0 && first_flag == false) target_degree += first_degree_diff;
					else if(first_flag == false){
						first_flag = true;
						initial_length = len_counter;
						target_degree = past_target_degree + first_diag_direction * 45.0;
						next_target_degree = target_degree;
					}

					if(len_counter >= len_measure(diag_length) - initial_length  && first_flag == true && last_flag == false && fabs(target_degree - next_target_degree) < 45.0) target_degree += last_degree_diff;

					if(len_counter >= len_measure(diag_length - 15.0 * (now_speed * now_speed - turn_speed * turn_speed) / 2.0 / (accel * 1000))) //conv to mm
						now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
					else
						now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
					read_encoder();
					add_coordinate(-sin(degree * PI /180.0),cos(degree * PI / 180.0));
					target_theta_now = (degree - target_degree * 1.01) / 180.0 * PI;
					speed_controller(now_speed,- (degree_p * target_theta_now) );
					target_theta_last = target_theta_now;
					ENCODER_start = OFF;
				}
			}
			target_degree = next_target_degree + last_diag_direction * 45.0;

			if(firstRunVec == lastRunVec)	RobotRunVec = firstRunVec;
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
			while(1){
				GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
				if(ENCODER_start == ON){
					read_encoder();
					add_coordinate(-sin(degree * PI /180.0),cos(degree * PI / 180.0));
					if(target_rad <= turn_speed / 90.0 && clothoid_flag == false){
						target_rad += turn_speed / 90.0 / 10.0;
						first_clothoid_degree = degree;
						speed_controller(turn_speed,(float)operation_direction * target_rad);
						clothoid_flag = false;
					}
					else{
						second_clothoid_degree = clothoid_flag == false ? target_degree + (float)operation_direction * (current_degree - first_clothoid_degree) : second_clothoid_degree;
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
		/*	else if(root[*i].op ==  Operation::TURN_LEFT90){
				const float current_degree = degree;
				target_direction += 1;
				static float target_rad = 0;
				float first_clothoid_degree = 0;
				float second_clothoid_degree = 0;
				bool clothoid_flag = false;
				int16_t target_degree = 0;
				if(root[(*i)+1].op == root[*i].op){
				target_direction += 1;
				(*i)++;
				start_buzzer(10);
				target_degree = target_direction * 90 - 4 * speed /50;
				}
				else
				target_degree = target_direction * 90 - 4 * speed / 100;
				while(1){
				if(ENCODER_start == ON){
				read_encoder();
				if(target_rad <= speed / 90.0 && clothoid_flag == false){
				target_rad += speed / 90.0 / 5.0;
				first_clothoid_degree = degree;
				speed_controller(speed,target_rad);
				clothoid_flag = false;
				}
				else{
				second_clothoid_degree =  clothoid_flag == false ? target_degree + (current_degree - first_clothoid_degree) : second_clothoid_degree;
				clothoid_flag = true;
				speed_controller(speed,target_rad);
				if(degree >= second_clothoid_degree)	target_rad -= speed / 90.0 / 5.0;
				if(target_rad <= 0)	break;
				}
				ENCODER_start=OFF;
				}
				}
				}*/
		stop_buzzer();
	}
	void Robot::action(uint8_t value,OperationList runSequence,ParamList parameters){
		for(size_t i = 0;i<runSequence.size();i++){
			len_counter = 0;
			robotShortMove(runSequence,parameters[value],&i);
		}
		Delay_ms(100);
		degree = 0;
		reset_e();
		len_counter = 0;
	}
