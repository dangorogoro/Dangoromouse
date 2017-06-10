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
	Direction WallData = read_wall(NORTH);
	agent->update(Robot::getRobotVec(),WallData);
	while(len_counter <= len_measure(140)){
		go_straight(0.01 * degree);
	}
	Robot::addRobotVec(IndexVec::vecNorth);
	len_counter = 0;
	set_speed(0,0);
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
void Robot::robotMove(Direction Nextdir){
	len_counter = 0;
	Direction Nowdir =  Robot::getRobotDir();
	if(Nowdir==NORTH){
		if(Nextdir==NORTH){
			while(len_counter <= len_measure(ONE_BLOCK)){
				const float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI;
				go_straight(target_theta);
			}
		}
		if(Nextdir==EAST){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90 );
		}
		if(Nextdir==WEST){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90 );
		}
		if(Nextdir==SOUTH){
			turn_back(getRobotDegreeDir());
			setRobotDegreeDir(2);
			start_wall(getRobotDegreeDir());
		}
	}
	if(Nowdir==WEST){
		if(Nextdir==NORTH){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90);
		}
		if(Nextdir==EAST){
			turn_back(getRobotDegreeDir());
			setRobotDegreeDir(-1);
			start_wall(getRobotDegreeDir());
		}
		if(Nextdir==WEST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				const float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI;
				go_straight(target_theta);
			}
		}
		if(Nextdir==SOUTH){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90);
		}
	}
	if(Nowdir==SOUTH){
		if(Nextdir==NORTH){
			turn_back(getRobotDegreeDir());
			setRobotDegreeDir(0);
			start_wall(getRobotDegreeDir());
		}
		if(Nextdir==EAST){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90);
		}
		if(Nextdir==WEST){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90);
		}
		if(Nextdir==SOUTH){
			while(len_counter <= len_measure(ONE_BLOCK)){
				const float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI;
				go_straight(target_theta);
			}
		}
	}
	if(Nowdir==EAST){
		if(Nextdir==NORTH){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90);
		}
		if(Nextdir==EAST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				const float target_theta =  (degree - getRobotDegreeDir() * 90) / 180.0 * PI;
				go_straight(target_theta);
			}
		}
		if(Nextdir==WEST){
			turn_back(getRobotDegreeDir());
			setRobotDegreeDir(1);
			start_wall(getRobotDegreeDir());
		}
		if(Nextdir==SOUTH){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90);
		}
	}
}
void Robot::robotShortMove(OperationList root,Param param,size_t *i){
	//USART_printf("x-- %d y--%d\r\n",RobotVec.x,RobotVec.y);
	//USART_printf("Run %d %d %d %d\r\n",RobotRunVec(0,0),RobotRunVec(0,1),RobotRunVec(1,0),RobotRunVec(1,1));
	len_counter = 0;
	static float target_degree = 0.0f;

	const static uint16_t curving_length = param.get_turn_param() / 30; // 60
	static int16_t now_speed = (left_speed + right_speed) / 2 / MmConvWheel;
	const static int16_t last_speed = param.get_last_param();
	const static int16_t turn_speed = param.get_turn_param();
	const static int16_t accel = param.get_accel_param();
	uint16_t length = 0;
	length = *i == 0 ? 130 + (root[*i].n - 1) * ONE_BLOCK - curving_length : root[*i].n * ONE_BLOCK - curving_length; //130 was
	
	float e_now = 0,e_sum = 0;
	float target_theta_now = 0,target_theta_last = 0;
	float x_p = 0.2 * 600.0 / last_speed;
	float degree_p = 40.0 * 600.0 / last_speed;
	float degree_d = 1.0 * 600.0 / last_speed;

	setRobotVecFromRun(root[*i].op,root[*i].n);
	if(root[*i].op == Operation::FORWARD){
		while(len_counter <= len_measure(length)){
			if(now_speed >= 1.0){
				x_p = 0.2 * 600.0 / now_speed;
				degree_p = 40.0 * 600.0 / now_speed;
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
				target_theta_now = (degree - target_degree * 1.01) / 180.0 * PI;
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

		int8_t first_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;
		target_degree += first_diag_direction * 45;
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
		GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET);
		int8_t last_diag_direction = (root[*i].op == Operation::TURN_RIGHT45) ? -1 : 1;

		while(len_counter <= len_measure(diag_length)){
			degree_p = 50.0;
			if(ENCODER_start == ON){
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
		target_degree += last_diag_direction * 45;
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
					second_clothoid_degree =  clothoid_flag == false ? target_degree + (float)operation_direction * (current_degree - first_clothoid_degree) : second_clothoid_degree;
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
