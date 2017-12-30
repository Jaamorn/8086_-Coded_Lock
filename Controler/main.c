#include "common.h"
#include "include.h"

void output_pw1();
void output_pw2();
void output_pw3();
void output_timebar();
void count_down(byte t_count);

uint8 pw1[6] = {'1', '2', '3', '4', '5', '6'};
uint8 pw2[6] = {'1', '2', '3', '3', '2', '1'};
uint8 pw3[6] = {'6', '5', '4', '3', '2', '1'};
uint8 countdown[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
uint8 pw_flag = 1;
uint8 timeset = 9;

void main(void) {
  DisableInterrupts;
  initialization();
  EnableInterrupts; //数据采集完成
  LCD_P6x8Str(10, 0, "Password Reset in");
  while (1) {
    output_pw1();
    count_down(9);
    gpio_turn(PTD8);
    DELAY_MS(800);
    gpio_turn(PTD8);

    pw_flag++;
    output_pw2();
    count_down(9);
    gpio_turn(PTD8);
    DELAY_MS(800);
    gpio_turn(PTD8);

    pw_flag++;
    output_pw3();
    count_down(9);
    gpio_turn(PTD8);
    DELAY_MS(800);
    gpio_turn(PTD8);
    pw_flag = 1;
  }
} // main
void output_pw1() {
  for (int i = 0; i <= 5; i++) {
    LCD_write_char(10 * i + 35, 6, pw1[i]);
  }
}
void output_pw2() {
  {
    for (int i = 0; i <= 5; i++)
      LCD_write_char(10 * i + 35, 6, pw2[i]);
  }
}
void output_pw3() {
  for (int i = 0; i <= 5; i++) {
    LCD_write_char(10 * i + 35, 6, pw3[i]);
  }
}
void count_down(uint8 t_count) {

  while (t_count != 0) {
    LCD_write_char(60, 2, countdown[t_count]);
    t_count--;
    DELAY_MS(1000);
  }
  t_count = timeset;
}
