#ifndef _KEY_BOARD_HPP_
#define _KEY_BOARD_HPP_

#include <wiringPi.h>
#include <iostream>
#include <chrono>


// Wpi of keys and switchs
#define K1_P 3
#define K2_P 4
#define K3_P 6

#define K4_P 5
#define K5_P 7
#define K6_P 8

#define K7_P 24
#define K8_P 26
#define K9_P 27

#define SW1_A_P 20
#define SW1_B_P 22

#define SW2_A_P 23
#define SW2_B_P 25

//
#define K1 1
#define K2 2
#define K3 3
#define K4 4
#define K5 5
#define K6 6
#define K7 7
#define K8 8
#define K9 9

#define SW1 10
#define SW2 12

//
#define SW_A_ON 0
#define SW_B_ON 1
#define SW_MID 2

#define SW_ERROR -1


void key_board_init();
bool key_pressed_down(const int &key_index, const std::chrono::high_resolution_clock::time_point &tp);
void key_update_last_pressed(const std::chrono::high_resolution_clock::time_point &tp);
int get_switch_status(const int &switch_index);

#endif
