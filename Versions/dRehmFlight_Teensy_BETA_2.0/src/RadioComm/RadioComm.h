//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

#ifndef RadioComm_h
#define RadioComm_h

#include <Arduino.h>

// Include all possible radio libraries
#include "../SBUS/SBUS.h"
// #include "sbus.h" // Bolder Flight Systems SBUS library -- had issues with loop timing using this library
#include <PPMReader.h>
#include "../DSMRX/DSMRX.h"

enum RadioType {
    PPM,
    DSM,
    sBUS
    // Removed PWM option because 1) it's not very popular anymore, and 2) The ISRs required to make it work were difficult to implement.
};

class RadioComm {
public:
    RadioComm(RadioType type);
    RadioComm(RadioType type, uint8_t num_channels);

    void begin();            //DSM & //SBUS
    void begin(int PPM_Pin); //PPM

    void getCommands(unsigned int* returnArray);
    void setFailsafe(unsigned int* failsafeArray);
    void failSafe();

    unsigned long channel_pwm[16] = {0}; 
private:
	RadioType _type;

    // bfs::SbusRx* _sbus;
    // bfs::SbusData _sbus_data;
    SBUS* _sbus;
    DSMRX* _dsm;
    PPMReader* _ppm;

    uint8_t   _num_channels = 6;    
    uint16_t _sbusChannels[16] = {0};
    unsigned int _channel_pwm_prev[4] = {0};
    unsigned int _channel_fs[16] = {0};

    bool _sbusFailSafe;
    bool _sbusLostFrame;

    //sBus scaling below is for Taranis-Plus and X4R-SB
    const float _sbus_scale = 0.615;  
    const float _sbus_bias  = 895.0; 
};


#endif