

// RadioComm.h
// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.


#ifndef RadioComm_h
#define RadioComm_h

#include <Arduino.h>

// Include all possible radio libraries
#include "../SBUS/SBUS.h"
// #include "sbus.h" // Bolder Flight Systems SBUS library
#include <PPMReader.h>
#include "../DSMRX/DSMRX.h"
#include "RC_Receiver.h"


enum RadioType {
    PWM,
    PPM,
    DSM,
    sBUS
};



class RadioComm {
public:
    RadioComm(RadioType type);
    RadioComm(RadioType type, uint8_t num_channels);




    void begin();      //DSM & //SBUS
    void begin(int PPM_Pin); //PPM
    void begin(int channelPins[]);   //PWM

    void getCommands(unsigned long int* returnArray);

    

    unsigned long channel_pwm[16] = {0}; 






private:
	RadioType _type;

    // bfs::SbusRx* _sbus;
    // bfs::SbusData _sbus_data;
    SBUS* _sbus;
    DSMRX* _dsm;
    PPMReader* _ppm;
    RC_Receiver* _pwm;

    uint8_t   _num_channels = 6;    
    uint16_t _sbusChannels[16] = {0};
    int _pwm_channel_pins[8] = {0};
    unsigned long _channel_pwm_prev[4] = {0};



    bool _sbusFailSafe;
    bool _sbusLostFrame;

    //sBus scaling below is for Taranis-Plus and X4R-SB
    const float _sbus_scale = 0.615;  
    const float _sbus_bias  = 895.0; 




};


#endif