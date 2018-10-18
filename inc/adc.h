#ifndef MY_ADC_H
#define MY_ADC_H
/*
   extern uint16_t race_sensor0;
   extern uint16_t race_sensor1;
   extern uint16_t race_sensor2;
   extern uint16_t race_sensor3;
   extern uint16_t line_color0;
   extern uint16_t line_color1;
   extern uint16_t line_color2;
   extern uint16_t line_color3;
   */
void ADC_setting();
void battery_check();
void start_ADC();
void check_line();
#define B 0
#define W 1 
#define boader_line 3000
#endif
