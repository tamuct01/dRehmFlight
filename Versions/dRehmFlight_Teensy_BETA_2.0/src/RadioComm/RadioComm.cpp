
// RadioComm.cpp
// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.



#include "RadioComm.h"



RadioComm::RadioComm(RadioType type): RadioComm(type, 6) {};

RadioComm::RadioComm(RadioType type, uint8_t num_channels) {
    _sbus = nullptr;
    // _sbus_data = nullptr;
    _dsm  = nullptr;
    _ppm  = nullptr;
    _pwm  = nullptr;

    _type = type;

    _num_channels = num_channels;
}






void RadioComm::begin() {
    if (_type == sBUS) {
        // _sbus = new bfs::SbusRx(&Serial5, true, false);
        // bfs::SbusData _sbus_data;
        // _sbus->Begin();

        _sbus = new SBUS(Serial5);
        _sbus->begin();




    }
    
    else if (_type == DSM) {
        _dsm = new DSM1024();
        Serial3.begin(115000);

    }

    else {
        Serial.println("No Serial RX type defined...");
        while(1);
    }
}

void RadioComm::begin(int PPM_Pin) {

    if (_type == PPM) {
        // _ppm = new PPM();
        // _ppm->begin(PPM_Pin, false);
        // ppm.begin(PPM_Pin, false);
        PPMReader _ppm(PPM_Pin, _num_channels);


    }

    else {
        Serial.println("No PPM type defined...");
        while(1);
    }
}


void RadioComm::begin(int channelPins[]) {
    byte i;

    // Assign channel pins into the Class variable.  _num_channels MUST be set at invocation time or the default of 6 will be used.
    for (i=0; i<_num_channels; i++) {
        _pwm_channel_pins[i] = channelPins[i];
    }

    if (_type == PWM) {
        RC_Receiver _pwm(_pwm_channel_pins[0], _pwm_channel_pins[1], _pwm_channel_pins[2], _pwm_channel_pins[3], 
            _pwm_channel_pins[4], _pwm_channel_pins[5], _pwm_channel_pins[6], _pwm_channel_pins[7]);

        
        
    }

    else {
        Serial.println("No PWM type defined...");
        while(1);
    }
}
    




void RadioComm::getCommands(unsigned long int* returnArray) {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

   int8_t i;
    if (_type == sBUS) {
        // _sbus->Read();

        // /* Grab the received data */
        // _sbus_data = _sbus->data();
        // /* Display the received data */
        // for (i = 0; i < _num_channels; i++) {
        //     channel_pwm[i] = _sbus_data.ch[i]* _sbus_scale + _sbus_bias; 
        // }
        // _sbusFailSafe = _sbus_data.failsafe;
        // _sbusLostFrame = _sbus_data.lost_frame;


        if (_sbus->read(_sbusChannels, &_sbusFailSafe, &_sbusLostFrame)) {
            for (i = 0; i < _num_channels; i++) {
                channel_pwm[i] = _sbusChannels[i] * _sbus_scale + _sbus_bias;
            }
        }






    }






//   #if defined USE_PPM_RX || defined USE_PWM_RX
//     channel_1_pwm = getRadioPWM(1);
//     channel_2_pwm = getRadioPWM(2);
//     channel_3_pwm = getRadioPWM(3);
//     channel_4_pwm = getRadioPWM(4);
//     channel_5_pwm = getRadioPWM(5);
//     channel_6_pwm = getRadioPWM(6);
    
//   #elif defined USE_DSM_RX
//     if (DSM.timedOut(micros())) {
//         //Serial.println("*** DSM RX TIMED OUT ***");
//     }
//     else if (DSM.gotNewFrame()) {
//         uint16_t values[num_DSM_channels];
//         DSM.getChannelValues(values, num_DSM_channels);

//         channel_1_pwm = values[0];
//         channel_2_pwm = values[1];
//         channel_3_pwm = values[2];
//         channel_4_pwm = values[3];
//         channel_5_pwm = values[4];
//         channel_6_pwm = values[5];
//     }
//   #endif

    // Low-pass the critical commands and update previous values
    float b = 0.7; //Lower=slower, higher=noiser

    for (i = 0; i < 4; i++) {
        channel_pwm[i] = (1.0 - b)*_channel_pwm_prev[i] + b*channel_pwm[i];
        _channel_pwm_prev[i] = channel_pwm[i];
    }

    // return the values to the function
    for (i = 0; i < _num_channels; i++) {
        returnArray[i] = channel_pwm[i];
    }

}


