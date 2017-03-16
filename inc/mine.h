#define MY_MINE_H
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "float.h"

#ifdef __cplusplus
#include "Maze.h"
#include "Agent.h"
#include "mazeData.h"
#include <vector>

#include "mazesolve.h"
extern "C" {
#endif /* __cplusplus */
#include "config.h"
#include "delay.h"
#include "timer.h"
#include "mouse_moving.h"
#include "servo.h"
#include "my_usart.h"
#include "adc.h"
#include "mpu6500.h"
#include "sensor.h"
#ifdef __cplusplus
}
#endif /* __cplusplus */
