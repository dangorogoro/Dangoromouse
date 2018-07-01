#ifndef MY_FLAG_H
#define MY_FLAG_H
#include "mine.h"
class Flag{
	private:
		bool save_maze;
	public:
		Flag() : save_maze{false}{}
		void setSaveMaze(bool flag){save_maze = flag;}
		bool getSaveMaze(){return save_maze;}
};
#endif
