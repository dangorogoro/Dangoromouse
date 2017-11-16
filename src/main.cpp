#include "mine.h"
int main(){
	GPIO_setting();
	TIMER_setting();
	SysTickTimer_Config();

	//////////////////
	/*
	
  
	
	while(1){
		Delay_ms(100);
	}
	SPI_setting();
	WriteReg(0x6B,0x80);
	Delay_ms(100);
	USART_printf("WHOAM!!---%d\r\n",ReadReg(117));
	WriteReg(0x6B,0x00);
	Delay_ms(100);
	WriteReg(0x1A,0x00);
	Delay_ms(100);
	WriteReg(0x1B,0x18);
	while(1){
		USART_printf("Z---%d\r\n",ReadGYRO());
		Delay_ms(10);
	}
	*/
	
	//////
	while(button_return == 0){
	}
	mouse_motor_setting();
	ADC_setting();
	USART_setting();
	encoder_setting();
	SPI_setting();
	MPU6500_setting();
	battery_check();
	TIM_Cmd(TIM5,ENABLE);
	GYRO_offset();
	
	Delay_ms(1000);
	mode_select = encoder_paramset();
	pipi(3);
	pipi(4);
	pipi(5);
	pipi(6);
	Delay_ms(1000);

	led_flash_setting();
	GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
	uint8_t param_value = encoder_paramset();
	TIM2->CNT = 0;
	TIM8->CNT = 0;
	Delay_ms(1000); //veryvery important

	ParamList parameters;
	parameters.setting();

	mouse_start();
	GYRO_offset();
	Delay_ms(100);
	pipi(3);
	pipi(4);
	pipi(5);
	pipi(6);

	Maze maze,maze_backup;
	Robot dango;
	Agent agent(maze);
	Agent::State prev_State = Agent::IDLE;

	while(1){
		if(mode_select % 10 == 0){
			led_flash();
			while(1){
				if(SENSOR_reset == ON){
					read_wall(0x00);
					USART_printf("%wall 1-> %d 2-> %d\r\n",led_1,led_2);
					USART_printf("%wall 3-> %d 4->%d\r\n",led_3,led_4);
					reset_led();
					SENSOR_reset=OFF;
				}
				if(SENSOR_start == ON)
					led_get();
			}
		}
		else if(mode_select % 10 == 1){
			while(1){
				if(ENCODER_start == ON){
					read_encoder();
					speed_controller(param_value*100,0);
					ENCODER_start = OFF;
				}
			}
		}
		else if(mode_select % 10 == 2){
			while(1){
				if(GYRO_start == ON){
					GYRO_sampling();
					GYRO_start=OFF;
				}
				if(ENCODER_start==ON){
					read_encoder();
					set_speed(-(left_speed+right_speed)/2.0f*0.9f+speed,-speed-(left_speed+right_speed)/2.0f*0.9f);
					ENCODER_start = OFF;
				}
				
				if(degree >= 90.0)
					GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
				else
					GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET);
			}
		}
		else if(mode_select % 10 == 3){
			stop_flag = false;
			led_flash();
			Delay_ms(1000);
			dango.startOffSet(&agent);
			prev_State = agent.getState();
			while(1){
				prev_State = agent.getState();
				Direction Lastdir = agent.getNextDirection();
				reset_led();
				sensor_works();
				Direction WallData = read_wall(dango.getRobotDir());
				if(stop_flag == true){
					pipi(4);
					set_speed(0,0);
					break;
				}
				if((maze.getWall(dango.getRobotVec().x,dango.getRobotVec().y) & Direction(0b11110000)) != (Direction)0xf0)
					agent.update(dango.getRobotVec(),WallData);
				else
					agent.update(dango.getRobotVec(),maze.getWall(dango.getRobotVec().x,dango.getRobotVec().y));
				if(agent.getState() == Agent::FINISHED){
					dango.startBack();
					break;
				}
				if(prev_State == Agent::SEARCHING_NOT_GOAL && agent.getState() != prev_State){
					maze_backup = maze;
					start_buzzer(10);
					led_fullon();
				}

				Direction Nextdir = agent.getNextDirection();
				if(Lastdir == Nextdir && len_counter >= 170)	len_counter -= 180.0;
				else len_counter = 0;

				if(Nextdir.byte == 0){
					set_speed(0,0);
					Delay_ms(1000);
					break;
				}
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

			Delay_ms(100);
			dango.setRobotVec(IndexVec());

			//agent.caclRunSequence(true);
			
			if(stop_flag == true){
				pipi(2);
				pipi(5);
				pipi(6);
				pipi(12);
				led_fullon();
				agent.resumeAt(Agent::FINISHED,maze_backup);
				while(button_return == 0){}
				pipi(4);
				mouse_start();
				led_fulloff();
			}
			agent.caclRunSequence(false);
			Robot last_dango = dango;
			OperationList runSequence = agent.getRunSequence();
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::STOP,1});
			runSequence = rebuildOperation(runSequence,0);
			TIM_Cmd(TIM5,ENABLE);
			while(1){
				dango.action(param_value,runSequence,parameters);
				param_value = encoder_paramset();
				plot.all_print();
				mouse_start();
				TIM2->CNT = 0;
				TIM8->CNT = 0;
				pipi(3);
				pipi(4);
				pipi(5);
				pipi(6);
				dango = last_dango;
				plot.clear();
			}
		}
		else if(mode_select % 10 == 4){
			Robot dango;
			Robot last_dango = dango;
			pipi(3);
			pipi(4);
			pipi(5);
			pipi(6);
			Delay_ms(1000);
			OperationList runSequence; 
			runSequence.push_back({Operation::FORWARD,3});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,2});
			runSequence.push_back({Operation::TURN_RIGHT90S,1});
			runSequence.push_back({Operation::TURN_RIGHT90S,1});
			runSequence.push_back({Operation::TURN_LEFT90S,1});
			runSequence.push_back({Operation::TURN_LEFT90S,1});
			runSequence.push_back({Operation::TURN_RIGHT90S,1});
			runSequence.push_back({Operation::TURN_RIGHT90S,1});
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,1});
			/*
			runSequence.push_back({Operation::FORWARD,3});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::TURN_RIGHT45,1});
			runSequence.push_back({Operation::FORWARD_DIAG,1});
			runSequence.push_back({Operation::TURN_RIGHT45,1});
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::TURN_RIGHT90,1});
			runSequence.push_back({Operation::FORWARD,2});
			*/
			runSequence.push_back({Operation::STOP,1});
			runSequence = rebuildOperation(runSequence,0);
			while(1){
				dango.action(param_value,runSequence,parameters);
				param_value = encoder_paramset();
				plot.all_print();
				TIM2->CNT = 0;
				TIM8->CNT = 0;
				mouse_start();
				pipi(3);
				pipi(4);
				pipi(5);
				pipi(6);
				dango = last_dango;
			}
		}
		else if(mode_select % 10 == 5){
			led_flash();
			while(1){
				sensor_works();
				if(ENCODER_start == ON){
					float target_speed = 0.0,target_rad = 0.0;
					int16_t target_left_value = 2603,target_right_value = 2800;
					// target_left_value = 2603,target_right_value = 2800; center
					float speed_gain = 0.3,rad_gain = 0.005;
					read_encoder();
					if(led_3 >= led_3_threshold && led_4 >= led_4_threshold){
						target_rad = rad_gain * (led_3 - target_left_value - (led_4 - target_right_value));
						target_speed = speed_gain * (led_3 - target_left_value + (led_4 - target_right_value));
					}
					speed_controller(-target_speed,target_rad);
					ENCODER_start = OFF;
					reset_led();
				}
			}
		}
		else if(mode_select % 10 == 6){
			Robot dango;
			for(int i = 0;i <= 10;i++){
				IndexVec po(0,i);
				agent.update(po,0b11110000);

				/*agent.update(IndexVec(1,2),0b1010);
					agent.update(IndexVec(1,3),0b0010);
					agent.update(IndexVec(1,4),0b1001);
					agent.update(IndexVec(2,4),0b0011);
					agent.update(IndexVec(2,3),0b1010);
					agent.update(IndexVec(2,2),0b1010);
					agent.update(IndexVec(2,1),0b1010);
					agent.update(IndexVec(2,0),0b0110);*/
				USART_printf("maze-data %d\r\n",maze.getWall(0,i));
			}
			agent.caclRunSequence(true);
			OperationList runSequence = agent.getRunSequence();
			runSequence.push_back({Operation::FORWARD,1});
			runSequence.push_back({Operation::STOP,1});
			dango.action(param_value,runSequence,parameters);
			GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
					GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET);

			while(1);
			
		}


	}
	return 0;
}
