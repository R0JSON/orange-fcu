#pragma once
#include <stdint.h> 
#define wiktor_setup

//Board config
/*
#ifdef bartek_setup
    #define BMP280
    #define LSM6DS3
    uint8_t pyro_channel_pin_1 = 2;
    uint8_t pyro_channel_pin_2 = 3;
    uint8_t servo_pin_0 = 5;
    uint8_t servo_pin_1 = 6;
    uint8_t servo_pin_2 = 7;
    uint8_t servo_pin_3 = 8;
    int buzzerPin = 4;  
#endif
*/
#ifdef wiktor_setup
    #define BMP280
    #define MPU9255
    #define SD_SWITCHPIN 34
    #define CS  13 
    const uint8_t pyro_channel_pin_1 = 26;
    const uint8_t pyro_channel_pin_2 = 25;
    const uint8_t servo_pin_0 = 4;
    const uint8_t servo_pin_1 = 15;
    const uint8_t servo_pin_2 = 32;
    const uint8_t servo_pin_3 = 33;
    const int buzzerPin = 2; 
#endif