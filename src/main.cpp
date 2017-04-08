#include "mine.h"
/*int main(void){
	SystemInit();
	GPIO_setting();
	TIMER_setting();
	set_servo();
	motor_setting();
//	ADC_setting();
	USART_setting();
	z_point=19;
	while (1){
		
		USART_printf("hello%d\n",z_point);
		Delay_ms(100);
		check_button_status();
		
		if(button_forward==ON)
			go_stright();
		else if(button_back==ON)
			go_back();
		else if(button_left==ON)
			turn_left();
		else if(button_right==ON)
			turn_right();
		else
			stop_motor();
		
		if(button_1==ON){
			if(button_b==ON)
				release_servo();
			else
				get_servo();
		}
		else if(button_2==ON){
			if(button_b==ON)
				z_point++;
			else 
				z_point--;
			z_servo();
		}
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
			GPIO_WriteBit(GPIOC,GPIO_Pin_1,Bit_RESET);
		else
			GPIO_WriteBit(GPIOC,GPIO_Pin_1,Bit_SET);

	}
	return 0;
}*/
int main(){
	GPIO_setting();
	SystemInit();
	TIMER_setting();
	SysTickTimer_Config();
	while(button_return==0){
		if(button_a==1){
			GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET);
			mode_select += 1;
			pipi(mode_select);
			while(button_a==1);
			Delay_ms(100);
		}
		GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_RESET);
	}
	Delay_ms(100);
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_SET);
	mouse_motor_setting();
	ADC_setting();
	USART_setting();
	encoder_setting();
	SPI_setting();
	MPU6500_setting();
	battery_check();
	TIM_Cmd(TIM5,ENABLE);
	GYRO_offset();
	Delay_ms(100);
	
	led_flash_setting();
	GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
	const uint8_t param_value = encoder_paramset();
	TIM2->CNT = 0;
	TIM8->CNT = 0;
	Delay_ms(1000); //veryvery important


	Maze maze,maze_backup;
	Agent agent(maze);
	Agent::State prev_State = Agent::IDLE;
	Robot dango;
	agent.getState();
	
	Param param1(0,100,100,10);
	Param param2(0,300,300,10);
	Param param3(0,600,600,10);
	Param param4(0,1000,500,10);
	Param param5(0,900,900,5);
	Param param6(0,1000,1000,5);
	ParamList parameters;
	parameters.push_back(param1);
	parameters.push_back(param2);
	parameters.push_back(param3);
	parameters.push_back(param4);
	parameters.push_back(param5);
	parameters.push_back(param6);
//
	while(1){
		if(mode_select%10==0){
			led_flash();
			while(1){
				if(SENSOR_reset==ON){
					read_wall(0x00);
					reset_led();
					SENSOR_reset=OFF;
				}
				if(SENSOR_start==ON)
					led_get();
			}
		}
		else if(mode_select%10==1){
			while(1){
				if(ENCODER_start==ON){
					read_encoder();
					speed_controller(param_value*100,0);
					ENCODER_start=OFF;
				}
				/*if(len_counter>=4096*4*5){
					GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_RESET);
					set_speed(0,0);
					stop_motor();
					while(1);
				}*/
			}
		}
		else if(mode_select%10==2){
			while(1){
				if(GYRO_start==ON){
					GYRO_sampling();
					GYRO_start=OFF;
				}
				if(ENCODER_start==ON){
					read_encoder();
					set_speed(-(left_speed+right_speed)/2.0f*0.9f+speed,-speed-(left_speed+right_speed)/2.0f*0.9f);
					ENCODER_start = OFF;
				}
			}
		}
		else if(mode_select%10==3){
			//led_flash();
			dango.startOffSet(&agent);
			prev_State = agent.getState();
			while(1){
				sensor_works();
				start_buzzer(10);
				Direction WallData = read_wall(dango.getRobotDir());
				agent.update(dango.getRobotVec(),WallData);
				reset_led();
				if(agent.getState() == Agent::FINISHED){
					dango.setRobotVec(NORTH);
					set_speed(0,0);
					Delay_ms(500);
					turn_back();
					set_speed(0,0);
					reset_e();
					len_counter = 0;
					while(len_counter > len_measure(-140)){
						if(ENCODER_start==ON){
							read_encoder();
							speed_controller(-100,0);
							ENCODER_start=OFF;
						}
					}
					break;
				}
				if(prev_State == Agent::SEARCHING_NOT_GOAL && 
						(agent.getState() == Agent::SEARCHING_REACHED_GOAL || agent.getState() == Agent::BACK_TO_START)){
					start_buzzer(5);
					maze_backup = maze;
				}
				prev_State = agent.getState();
				Direction Nextdir = agent.getNextDirection();
				Delay_ms(1000);
				start_buzzer(20);
				degree = 0;
				reset_e();
				len_counter = 0;
				dango.robotMove(Nextdir);
				dango.setRobotDir(Nextdir);
				dango.addRobotDirToVec(Nextdir);
				set_speed(0,0);
				stop_buzzer();
			}
			start_buzzer(3);
			set_speed(0,0);
			len_counter = 0;
			reset_e();
			agent.caclRunSequence(false);
			const OperationList &runSequence = agent.getRunSequence();
			pipi(3);
			pipi(4);
			pipi(5);
			pipi(6);
			Delay_ms(1000);
			while(1){
				for(size_t i = 0;i<=runSequence.size();i++){
					len_counter = 0;
					dango.robotShortMove(runSequence,parameters[param_value],&i);
				}
				start_buzzer(100);
				while(1);
			}
		}
		else if(mode_select % 10 == 4){
			pipi(3);
			pipi(4);
			pipi(5);
			pipi(6);
			Delay_ms(1000);
			OperationList runSequence; 
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::TURN_LEFT90,1});
			runSequence.push_back({Operation::FORWARD,2});
			runSequence.push_back({Operation::TURN_LEFT90,1});
			runSequence.push_back({Operation::TURN_LEFT90,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,3});
			runSequence.push_back({Operation::STOP,1});
			while(1){
				for(size_t i = 0;i<runSequence.size();i++){
					len_counter = 0;
					dango.robotShortMove(runSequence,parameters[param_value],&i);
				}
				start_buzzer(100);
				while(1);
			}
		}
		else if(mode_select % 10 == 5){
			while(1){
				if(ENCODER_start == ON){
					read_encoder();
					speed_controller(param_value*100,(float)param_value/2.0);
					ENCODER_start=OFF;
				}
			}
		}
	}
	return 0;
}

				/*else if(mode_select%10==3){
				while(degree*10.0<=90)
					set_speed((int16_t)(left_speed+(int16_t)(100-left_speed)*2.4f),-(int16_t)(right_speed+(int16_t)(100-right_speed)*2.0f));
				set_speed(0,0);
				degree=0;
			}*/
					//set_speed((int16_t)(left_speed+(int16_t)(100-left_speed)*2.4f)+speed,(int16_t)(right_speed+(int16_t)(100-right_speed)*2.0f)-speed);
