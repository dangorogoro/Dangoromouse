#ifndef MY_DOT_H
#define MY_DOT_H
typedef struct{
  float x;
  float y;
  float rad;
}dotData;
class Traject{
  private:
    std::vector<dotData> data;
  public:
    Traject(): data(){}
    void push_back(float y, float x, float rad);
    dotData get_data(uint32_t index){return data[index];}
};
extern Traject turn45;
void set_traject();
#endif
