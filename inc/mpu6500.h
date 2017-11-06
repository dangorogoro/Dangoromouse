#ifndef MY_MPU6500_H
#define MY_MPU6500_H
void SPI_setting();
void MPU6500_setting();
uint8_t SPI_exchange(uint8_t TX_Data);
uint8_t ReadReg(uint8_t reg);
void WriteReg(uint8_t reg, uint8_t data);
int16_t ReadGYRO();
int16_t ReadZAccel();
void GYRO_offset();
void GYRO_sampling();
bool checkZAccel();
extern float degree;
extern float degree_old;
extern float GYRO_offset_data;
extern volatile float GYRO_old;
extern volatile float GYRO_new;
extern volatile uint8_t GYRO_start;
extern int32_t speed;
#endif
