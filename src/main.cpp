#include "mine.h"
/*int main(void){
	SystemInit();
	GPIO_setting();
	TIMER_setting();
	set_servo();
	motor_setting();
	ADC_setting();
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
	while(button_return == 0){
		if(button_a == 1){
			GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET);
			mode_select += 1;
			pipi(mode_select);
			while(button_a == 1);
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
	Delay_ms(500);
	GYRO_offset();
	
	led_flash_setting();
	GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
	uint8_t param_value = encoder_paramset();
	TIM2->CNT = 0;
	TIM8->CNT = 0;
	Delay_ms(1000); //veryvery important


	Maze maze,maze_backup;
	Agent agent(maze);
	Agent::State prev_State = Agent::IDLE;
	Robot dango;
	ParamList parameters;
	parameters.setting();

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
		else if(mode_select % 10 == 3){
			led_flash();
			//Delay_ms(100);
			dango.startOffSet(&agent);
			prev_State = agent.getState();
			while(1){
				reset_led();
				sensor_works();
				Direction WallData = read_wall(dango.getRobotDir());
				agent.update(dango.getRobotVec(),WallData);
				if(agent.getState() == Agent::FINISHED){
					dango.setRobotVec(NORTH);
					set_speed(0,0);
					Delay_ms(500);
					turn_back(dango.getRobotDegreeDir());
					set_speed(0,0);
					reset_e();
					len_counter = 0;
					while(len_counter > len_measure(-180)){
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
					maze_backup = maze;
				}
				prev_State = agent.getState();
				Direction Nextdir = agent.getNextDirection();
				reset_e();
				len_counter = 0;
				dango.robotMove(Nextdir);
				dango.setRobotDir(Nextdir);
				dango.addRobotDirToVec(Nextdir);
				stop_buzzer();
			}
			degree = 0;
			set_speed(0,0);
			len_counter = 0;
			reset_e();
			pipi(3);
			pipi(4);
			pipi(5);
			pipi(6);
			led_stop();
			TIM_Cmd(TIM5,DISABLE);
			/*
			for(int i = 1;i <= 15;i ++){
				for(int j = 1;j <= 15;j ++){
					if(maze.getWall(i,j) / 16 != 15){
						IndexVec po(i,j);
						maze.updateWall(po,0b1111 ,true);
					}
				}
				for(int i = 0;i<=2;i++){
					for(int j =0;j<=2;j++){
						USART_printf("%d,%d---%d\r\n",i,j,maze.getWall(i,j));
						Delay_ms(200);
					}
				}
			}*/
			Delay_ms(100);
			IndexVec po(0,0);
			dango.setRobotVec(po);
			agent.caclRunSequence(true);
			Robot last_dango = dango;
			OperationList runSequence = agent.getRunSequence();
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::STOP,1});
			TIM_Cmd(TIM5,ENABLE);
			while(1){
				for(size_t i = 0;i<runSequence.size();i++){
					len_counter = 0;
					dango.robotShortMove(runSequence,parameters[param_value],&i);
				}
				TIM_Cmd(TIM5,DISABLE);
				param_value = encoder_paramset();
				pipi(3);
				pipi(4);
				pipi(5);
				pipi(6);
				Delay_ms(100);
				dango = last_dango;
				TIM_Cmd(TIM5,ENABLE);
				//while(1);
			}
		}
		else if(mode_select % 10 == 4){
			pipi(3);
			pipi(4);
			pipi(5);
			pipi(6);
			Delay_ms(1000);
			OperationList runSequence; 
			runSequence.push_back({Operation::FORWARD,4});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,4});
			/*
			for(int i = 0; i<= 3; i++){
				runSequence.push_back({Operation::FORWARD,3});
				runSequence.push_back({Operation::TURN_RIGHT90,1});
				runSequence.push_back({Operation::TURN_RIGHT90,1});
			}
			*/
			runSequence.push_back({Operation::STOP,1});
			while(1){
				for(size_t i = 0;i<runSequence.size();i++){
					len_counter = 0;
					dango.robotShortMove(runSequence,parameters[param_value],&i);
				}
				start_buzzer(10);
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
