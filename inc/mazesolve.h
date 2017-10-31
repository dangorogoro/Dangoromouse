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
		bool leftWall;
		bool rightWall;
		bool sideWall;
		bool frontWall;
	public:
		//Direction RobotRunVec = NORTH;
		Matrix2i RobotRunVec;
		Matrix2i getRIGHT(){Matrix2i po; po << 0, -1,1,0; return po;}
		Matrix2i getFORWARD(){Matrix2i po; po << 1,0,0,1; return po;}
		Matrix2i getLEFT(){Matrix2i po; po << 0, 1,-1,0; return po;}
		Matrix2i setRunVec(){Matrix2i po; po << 0, 1 ,0 ,0; return po;}
		Robot() : RobotDir{NORTH}, RobotDegreeDir{0}, x_point{0.0}, y_point{0.0}, RobotRunVec{setRunVec()}{}
		float centerDistance();
		void setSpeed();
		void setRobotVec(IndexVec vec);
		void addRobotVec(IndexVec vec);
		void setRobotDir(Direction dir);
		void addRobotDirToVec(Direction dir);
		void startOffSet(Agent* agent);
		void goLeft();
		void goBack(int8_t Nextdir);
		void goStraight();
		void goStraight(uint16_t length);
		void slalomStraight(uint16_t length);
		void goRight();
		void robotMove(Direction Nextdir);
		void robotShortMove(OperationList root,Param param,size_t *i);
		IndexVec getRobotVec();
		Direction getRobotDir();
		inline float x()const{return x_point;}
		inline float y()const{return y_point;}
		inline void set_x(float coordinate){ x_point = coordinate;}
		inline void set_y(float coordinate){ y_point = coordinate;}
		inline void add_x(float coordinate){ x_point += coordinate;}
		inline void add_y(float coordinate){ y_point += coordinate;}
		void set_coordinate(float coordinate_x, float coordinate_y);
		void add_coordinate(float degre);
		void addRobotDegreeDir(int8_t dir){RobotDegreeDir += dir;}
		void setRobotDegreeDir(int8_t dir){RobotDegreeDir	 = dir;}
		int8_t getRobotDegreeDir()const{return RobotDegreeDir;}
		float runningCoordinate();
		void setRobotVecFromRun(uint8_t Direction,uint8_t n);
		void setWallStatus();
		void action(uint8_t value,OperationList runSequence,ParamList param);
};
#endif
