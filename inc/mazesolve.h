#ifndef MY_MAZESOLVE_H
#define MY_MAZESOLVE_H
#include "mine.h"
#include "param.h"
class Robot{
	private:
		int16_t LeftEncoder,RightEncoder;
		IndexVec RobotVec;
		Direction RobotDir;
		int8_t RobotDegreeDir;
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
		inline void set_x(float coordinate){ x_point = coordinate;}
		inline void set_y(float coordinate){ y_point = coordinate;}
		inline void add_x(float coordinate){ x_point += coordinate;}
		inline void add_y(float coordinate){ y_point += coordinate;}
		void set_coordinate(float coordinate_x, float coordinate_y);
		void add_coordinate(float coordinate_x, float coordinate_y);
		void addRobotDegreeDir(int8_t dir){RobotDegreeDir += dir;}
		void setRobotDegreeDir(int8_t dir){RobotDegreeDir	 = dir;}
		int8_t getRobotDegreeDir()const{return RobotDegreeDir;}
};
#endif
