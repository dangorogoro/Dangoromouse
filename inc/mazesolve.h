#ifndef MY_MAZESOLVE_H
#define MY_MAZESOLVE_H
#include "mine.h"
#include "param.h"
#include "flag.h"
extern Maze maze,maze_backup;
struct Position{
  float x;
  float y;
};

class Robot{
  private:
    int16_t LeftEncoder,RightEncoder;
    IndexVec RobotVec;
    Direction RobotDir;
    int8_t RobotDegreeDir;
    struct Position position;
    bool leftWall;
    bool rightWall;
    bool sideWall;
    bool frontWall;
    bool zStatus;
    uint16_t left_sensor;
    uint16_t right_sensor;
    Flag flag;
  public:
    //Direction RobotRunVec = NORTH;
    Matrix2i RobotRunVec;
    Matrix2i setRunVec(){Matrix2i po; po << 0, 1 ,0 ,0; return po;}
    Robot() : RobotDir{NORTH}, RobotDegreeDir{0}, position{0,0},zStatus{false}, RobotRunVec{setRunVec()}{}
    inline void saveMazeStart(){flag.setSaveMaze(true);}
    inline bool getSaveMazeFlag(){return (flag.getSaveMaze());}
    inline bool getSearchingSaveFlag(){return (flag.getSearchingSave());}
    float centerDistance();
    void setSpeed();
    void setRobotVec(IndexVec vec);
    void addRobotVec(IndexVec vec);
    void setRobotDir(Direction dir);
    void addRobotDirToVec(Direction dir);
    void startOffSet(Agent* agent);
    void goLeft();
    void goBack(int8_t Nextdir, bool goal_flag);
    void goStraight();
    void goStraight(uint16_t length);
    void slalomStraight(uint16_t length);
    void goRight();
    void robotMove(Direction Nextdir, bool goal_flag);
    void robotShortMove(OperationList root,Param param,size_t *i);
    IndexVec getRobotVec();
    Direction getRobotDir();
    inline float x()const{return position.x;}
    inline float y()const{return position.y;}
    inline void set_x(float coordinate){ position.x = coordinate;}
    inline void set_y(float coordinate){ position.y = coordinate;}
    inline void add_x(float coordinate){ position.x += coordinate;}
    inline void add_y(float coordinate){ position.y += coordinate;}
    inline Position getPosition(){return position;}
    void fixCoordinate();
    void fixCoordinate(Matrix2i runVec, float offset);
    void fixCoordinate(Matrix2i runVec, float wall_left,float wall_right);
    inline void set_left_sensor(uint16_t sensor){ left_sensor = sensor;}
    inline void set_right_sensor(uint16_t sensor){ right_sensor = sensor;}
    inline uint16_t get_left_sensor(){ return left_sensor;}
    inline uint16_t get_right_sensor(){ return right_sensor;}
    void set_coordinate(float coordinate_x, float coordinate_y);
    void add_coordinate(float degre);
    void addRobotDegreeDir(int8_t dir){RobotDegreeDir += dir;}
    void setRobotDegreeDir(int8_t dir){RobotDegreeDir	 = dir;}
    int8_t getRobotDegreeDir()const{return RobotDegreeDir;}
    float runningCoordinate();
    void setRobotVecFromRun(uint8_t Direction,uint8_t n);
    void setWallStatus();
    void action(uint8_t value,OperationList runSequence,ParamList param);
    void startBack(Direction dir, bool reverse_flag);
    float targetLength(IndexVec targetIndex,Matrix2i vecStatus,uint16_t offset);
    bool judgeTargetCoordinate(IndexVec targetIndex, Matrix2i vecStatus,uint16_t offset);
    float centerDistance(IndexVec firstVec,IndexVec lastVec,Matrix2i vecStatus, Operation::OperationType op);
};

OperationList rebuildOperation(OperationList list,bool diagFlag);
OperationList reverseOperation(OperationList list);
OperationList reverseOperation(OperationList list,bool flag);
struct Position setStartPosition(Robot &dango);
Position estimatePosition(Position est);
inline Matrix2i right90Rotate(){Matrix2i po; po << 0, -1, 1, 0; return po;}
inline Matrix2i eigenRotate(){Matrix2i po; po << 1, 0, 0, 1; return po;}
inline Matrix2i left90Rotate(){Matrix2i po; po << 0, 1, -1, 0; return po;}
inline Matrix2i Rotate180(){Matrix2i po; po << -1, 0, 0, -1; return po;}
Matrix2i getRotate(Operation::OperationType type);
Direction directionFromRunVec(Matrix2i runVec);
bool judge_diag_turn(Operation::OperationType op);
Position getPositionFromVec(IndexVec vec, Direction dir);
Position getPositionFromVec(IndexVec vec);
#endif
