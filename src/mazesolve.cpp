#include "mine.h"
Robot::Robot(){
	RobotDir = NORTH;
	RobotDegreeDir = 0;
	x_point = 0.0;
	y_point = 0.0;
};
//use with read_encoder()
void Robot::setSpeed(){
	LeftEncoder 	= left_speed;
	RightEncoder = right_speed;
}
void Robot::setRobotVec(IndexVec vec){
	RobotVec	=	vec; 
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
void Robot::startOffSet(Agent *agent){
	sensor_works();
	Direction WallData = read_wall(NORTH);
	agent->update(Robot::getRobotVec(),WallData);
	while(len_counter <= len_measure(140)){
		go_straight();
	}
	Robot::addRobotVec(IndexVec::vecNorth);
	len_counter = 0;
	set_speed(0,0);
}
void Robot::robotMove(Direction Nextdir){
	len_counter = 0;
	Direction Nowdir =  Robot::getRobotDir();
	if(Nowdir==NORTH){
		if(Nextdir==NORTH){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
		if(Nextdir==EAST){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90 + 3);
		}
		if(Nextdir==WEST){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90 - 3);
		}
		if(Nextdir==SOUTH){
			turn_back();
			start_wall();
			setRobotDegreeDir(2);
			degree = 180.0;
		}
	}
	if(Nowdir==WEST){
		if(Nextdir==NORTH){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90 + 3);
		}
		if(Nextdir==EAST){
			turn_back();
			start_wall();
			setRobotDegreeDir(-1);
			degree = -90.0;
		}
		if(Nextdir==WEST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
		if(Nextdir==SOUTH){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90 - 3);
		}
	}
	if(Nowdir==SOUTH){
		if(Nextdir==NORTH){
			turn_back();
			start_wall();
			setRobotDegreeDir(0);
			degree = 0.0;
		}
		if(Nextdir==EAST){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90 - 3);
		}
		if(Nextdir==WEST){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90 + 3);
		}
		if(Nextdir==SOUTH){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
	}
	if(Nowdir==EAST){
		if(Nextdir==NORTH){
			addRobotDegreeDir(1);
			go_left(getRobotDegreeDir() * 90 - 3);
		}
		if(Nextdir==EAST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
		if(Nextdir==WEST){
			turn_back();
			start_wall();
			setRobotDegreeDir(1);
			degree = 90.0;
		}
		if(Nextdir==SOUTH){
			addRobotDegreeDir(-1);
			go_right(getRobotDegreeDir() * 90 + 3);
		}
	}
}
void Robot::robotShortMove(OperationList root,Param param,size_t *i){
	len_counter = 0;
	static int16_t target_direction = 0;
	const static uint8_t curving_length = param.get_last_param() / 3; // 60
	static int16_t now_speed = (left_speed + right_speed) / 2 / MmConvWheel;
	const static int16_t last_speed = param.get_last_param();
	const static int16_t turn_speed = param.get_turn_param();
	const static int16_t accel = param.get_accel_param();
	uint16_t length = 0;
	length =  *i == 0 ? 130 + (root[*i].n-1) * ONE_BLOCK - curving_length : root[*i].n*ONE_BLOCK - curving_length; //130 was
	
	if(root[*i].op == Operation::FORWARD){
		while(len_counter <= len_measure(length)){
			/*if(SENSOR_start==ON)
				led_get();
				if(SENSOR_reset==ON){
				read_wall(0x00);
				value = 0;
				if(led_1 >= 2200 && led_4 >= 2200)
				value = led_4 - led_1;
				SENSOR_reset=OFF;
				reset_led();
				}*/
			if(ENCODER_start==ON){
				if(len_counter >= len_measure(length-(now_speed*now_speed - turn_speed*turn_speed) /1000.0 / 2.0 / (accel * 1000))) //conv to mm
					now_speed = turn_speed >= now_speed ? turn_speed : now_speed - accel;
				else
					now_speed = last_speed <= now_speed ? last_speed : now_speed + accel;
				read_encoder();
				speed_controller(now_speed,0);
				ENCODER_start=OFF;
			}
		}
	}
	else if(root[*i].op == Operation::STOP){
		set_speed(0,0);
	}
	else if((root[*i].op == Operation::TURN_RIGHT90) || (root[*i].op  == Operation::TURN_LEFT90)){
		int8_t operation_direction = (root[*i].op == Operation::TURN_RIGHT90) ? -1 : 1;
		const float current_degree = degree;
		target_direction += operation_direction;
		float target_rad = 0;
		float first_clothoid_degree = 0;
		float second_clothoid_degree = 0;
		bool clothoid_flag = false;
		int16_t target_degree = 0;
		if(root[(*i)+1].op == root[*i].op){
			target_direction += operation_direction;
			(*i)++;
			target_degree = target_direction * 90 -(float)(operation_direction) * 3.5 * turn_speed / 50;
		}
		else
			target_degree = target_direction * 90 -(float)(operation_direction) * 3.5 * turn_speed / 50;
		while(1){
			GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
			if(ENCODER_start == ON){
				read_encoder();
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
					if(operation_direction == -1 && degree <= second_clothoid_degree)	target_rad -= turn_speed / 90.0 / 10.0 ;
					if(operation_direction == 1 && degree >= second_clothoid_degree)	target_rad -= turn_speed / 90.0 / 10.0 ;
					if(target_rad <= 0)	break;
				}
				ENCODER_start=OFF;
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
