#include "key_board.hpp"

int key_pins[14] = {200, K1_P, K2_P, K3_P, K4_P, K5_P, K6_P, K7_P, K8_P, K9_P, SW1_A_P, SW1_B_P, SW2_A_P, SW2_B_P};
bool last_pressed[10];
std::chrono::high_resolution_clock::time_point last_pressed_tp[10];

void key_board_init() {
    for (int i=1; i!=14; i++) {
        pinMode(key_pins[i], INPUT);  // 设置为输入模式
        pullUpDnControl(key_pins[i], PUD_UP);   // 上拉输入

    }
    
}

bool key_pressed_down(const int &key_index, const std::chrono::high_resolution_clock::time_point &tp) {
    if (key_index < 1 || key_index > 9) {
        std::cout << "key index error" << std::endl;

        return false;
    }

    if (digitalRead(key_pins[key_index]) == 0 && last_pressed[key_index] == false){
        last_pressed[key_index] = true;
        last_pressed_tp[key_index] = tp;

        return true;
    }
    else {
        return false;
    }
}

void key_update_last_pressed(const std::chrono::high_resolution_clock::time_point &tp) {
    for (int i=1; i != 10; i++) {
        if (last_pressed[i] && digitalRead(key_pins[i]) == 1 && std::chrono::duration_cast<std::chrono::microseconds>(tp - last_pressed_tp[i]).count() > 20000) {
            last_pressed[i] = false;
        }
    }
}

int get_switch_status(const int &switch_index) {
    if (switch_index != 10 && switch_index != 12) {
        std::cout << "switch index error" << std::endl;

        return SW_ERROR;
    }

    if (digitalRead(key_pins[switch_index]) == 1 && digitalRead(key_pins[switch_index+1]) == 0) {
        return SW_A_ON;
    }
    else if (digitalRead(key_pins[switch_index]) == 0 && digitalRead(key_pins[switch_index+1]) == 1) {
        return SW_B_ON;
    }
    else if (digitalRead(key_pins[switch_index]) == 1 && digitalRead(key_pins[switch_index+1]) == 1) {
        return SW_MID;
    }
    else {
        return SW_ERROR;
    }
}


