#ifndef MY_MINE_H
#define MY_MINE_H
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdarg.h"
//#include "float.h"
#include "math.h"
#define PI 3.14159

#ifdef __cplusplus
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;
#include "Maze.h"
#include "Agent.h"
#include "mazeData.h"
#include <vector>
#include "mazesolve.h"
#include "param.h"
#include "plot.h"
#include "flash.h"
#include <string>
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
#endif
