#include "mine.h"
Robot::Robot(){
	RobotDir = NORTH;
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
	SENSOR_reset=OFF;
	while(len_counter <= len_measure(135)){
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
		if(Nextdir==EAST)
			go_right();
		if(Nextdir==WEST)
			go_left();
		if(Nextdir==SOUTH){
			turn_back();
			start_wall();
		}
	}
	if(Nowdir==WEST){
		if(Nextdir==NORTH)
			go_right();
		if(Nextdir==EAST){
			turn_back();
			start_wall();
		}
		if(Nextdir==WEST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
		if(Nextdir==SOUTH)
			go_left();
	}
	if(Nowdir==SOUTH){
		if(Nextdir==NORTH){
			turn_back();
			start_wall();
		}
		if(Nextdir==EAST)
			go_left();
		if(Nextdir==WEST)
			go_right();
		if(Nextdir==SOUTH){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
	}
	if(Nowdir==EAST){
		if(Nextdir==NORTH)
			go_left();
		if(Nextdir==EAST){
			while(len_counter <= len_measure(ONE_BLOCK)){
				go_straight();
			}
		}
		if(Nextdir==WEST){
			turn_back();
			start_wall();
		}
		if(Nextdir==SOUTH)
			go_right();
	}
}
void Robot::robotShortMove(OperationList root,uint16_t speed,size_t *i){
	uint16_t length = 0;
	length =  *i == 0 ? 130+(root[*i].n-1)*ONE_BLOCK : root[*i].n*ONE_BLOCK;
	if(root[*i].op == Operation::FORWARD){
		while(len_counter <= len_measure(length)){
			static int16_t value = 0;
			if(SENSOR_start==ON)
				led_get();
			if(SENSOR_reset==ON){
				read_wall(0x00);
				value = 0;
				if(led_1 >= 2200 && led_4 >= 2200)
					value = led_4 - led_1;
				SENSOR_reset=OFF;
				reset_led();
			}
			if(ENCODER_start==ON){
				read_encoder();
				speed_controller(speed,0);
				ENCODER_start=OFF;
			}
		}
	}
	else if(root[*i].op == Operation::TURN_RIGHT90){
		static float target_rad = 0;
		float first_clothoid_degree = 0;
		float second_clothoid_degree = 0;
		bool clothoid_flag = false;
		int16_t target_degree = 0;
		if(root[(*i)+1].op == root[*i].op){
			(*i)++;
			start_buzzer(10);
			target_degree = -180 + 4 * speed / 50;
		}
		else
			target_degree = -90 + 4 * speed / 100;
		degree = 0;
		while(1){
			if(ENCODER_start == ON){
				read_encoder();
				if(target_rad <= speed / 90.0 && clothoid_flag == false){
					target_rad += speed / 90.0 / 10.0;
					first_clothoid_degree = degree;
					speed_controller(speed,-target_rad);
					clothoid_flag = false;
				}
				else{
					second_clothoid_degree =  clothoid_flag == false ? target_degree - first_clothoid_degree : second_clothoid_degree;
					clothoid_flag = true;
					speed_controller(speed,-target_rad);
					if(degree <= second_clothoid_degree)	target_rad -= speed / 90.0 / 10.0 ;
					if(target_rad <= 0)	break;
				}
				ENCODER_start=OFF;
			}
		}
		degree = 0;
		stop_buzzer();
	}
	else if(root[*i].op ==  Operation::TURN_LEFT90){
		static float target_rad = 0;
		float first_clothoid_degree = 0;
		float second_clothoid_degree = 0;
		bool clothoid_flag = false;
		int16_t target_degree = 0;
		if(root[(*i)+1].op == root[*i].op){
			(*i)++;
			start_buzzer(10);
			target_degree = 180 - 4 * speed /50;
		}
		else
			target_degree = 90 - 4 * speed / 100;
		degree = 0;
		while(1){
			if(ENCODER_start == ON){
				read_encoder();
				if(target_rad <= speed / 90.0 && clothoid_flag == false){
					target_rad += speed / 90.0 / 10.0;
					first_clothoid_degree = degree;
					speed_controller(speed,target_rad);
					clothoid_flag = false;
				}
				else{
					second_clothoid_degree =  clothoid_flag == false ? target_degree - first_clothoid_degree : second_clothoid_degree;
					clothoid_flag = true;
					speed_controller(speed,target_rad);
					if(degree >= second_clothoid_degree)	target_rad -= speed / 90.0 / 10.0;
					if(target_rad <= 0)	break;
				}
				ENCODER_start=OFF;
			}
		}
		degree = 0;
	}
}

