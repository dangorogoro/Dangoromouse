#ifndef MY_MAZESOLVE_H
#define MY_MAZESOLVE_H
class Robot{
	private:
		int16_t LeftEncoder,RightEncoder;
		IndexVec RobotVec;
		Direction RobotDir;
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
		void robotShortMove(OperationList root,uint16_t speed,size_t *i);
};
#endif
