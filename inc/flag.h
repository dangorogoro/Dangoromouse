#ifndef MY_FLAG_H
#define MY_FLAG_H
#include "mine.h"
class Flag{
  private:
    bool save_maze;
    uint8_t searching_save;
  public:
    Flag() : save_maze{false}, searching_save(3){}
    void setSaveMaze(bool flag){save_maze = flag;}
    bool getSaveMaze(){return save_maze;}
    uint8_t getSearchingSave(){searching_save = (searching_save + 1) % 4; return searching_save;}
};
#endif
