#ifndef MY_MAZESOLVE_H
#include "param.h"
#define MY_MAZESOLVE_H
class Robot{
	private:
		int16_t LeftEncoder,RightEncoder;
		IndexVec RobotVec;
		Direction RobotDir;
		int8_t RobotDirection;
		float x_point;
		float y_point;
	public:
		Robot();
		void setSpeed();
		void setRobotVec(IndexVec vec);
		void addRobotVec(IndexVec vec);
		IndexVec getRobotVec();
		Direction getRobotDir();
		void setRobotDir(Direction dir);
		void addRobotDirToVec(Direction dir);
		void startOffSet(Agent* agent);
		void robotMove(Direction Nextdir);
		void robotShortMove(OperationList root,Param param,size_t *i);
		inline float x()const{return x_point;}
		inline float y()const{return y_point;}
};
#endif
