
// RadioComm.cpp
// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.



#include "RadioComm.h"



RadioComm::RadioComm(RadioType type): RadioComm(type, 6) {};

RadioComm::RadioComm(RadioType type, uint8_t num_channels) {
    _sbus = nullptr;
    // _sbus_data = nullptr;
    _dsm  = nullptr;
    _ppm  = nullptr;

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
        _ppm = new PPMReader(PPM_Pin, _num_channels);
        // _ppm->begin(PPM_Pin, false);
        // ppm.begin(PPM_Pin, false);
        // PPMReader _ppm(PPM_Pin, _num_channels);


    }

    else {
        Serial.println("No PPM type defined...");
        while(1);
    }
}

    




void RadioComm::getCommands(unsigned int* returnArray) {
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

    else if (_type == PPM) {
        for (i=0; i < _num_channels; i++) {
            channel_pwm[i] = _ppm->latestValidChannelValue(i+1,1000);
        }
    }

    else if (_type == DSM) {
        if (_dsm->timedOut(micros())) {
            //Serial.println("*** DSM RX TIMED OUT ***");
        }
        else if (_dsm->gotNewFrame()) {
            // uint16_t values[_num_channels];
            _dsm->getChannelValues(_sbusChannels, _num_channels);

            for (i = 0; i < _num_channels; i++) {
                channel_pwm[i] = _sbusChannels[i];
            }
        }
    }



    else {
        // Should probably have an error here
    }
        

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


void RadioComm::setFailsafe(unsigned int* failsafeArray) {
    for (int8_t i=0; i<_num_channels; i++) {
        _channel_fs[i] = failsafeArray[i];
    }
}





void RadioComm::failSafe() {
    //DESCRIPTION: If radio gives garbage values, set all commands to default values
    /*
    * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
    * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
    * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
    * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
    * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
    * 
    * When using SBUS, the library handles the detection of lost frames and failsafe.  Some receivers handle failsafe differently.
    * Testing with FrSky SBUS receivers immediately triggered failsafe when Tx disconnected.  A Radiomaster RP3-H ELRS receiver would
    * not trigger failsafe when set to "No Pulses", but would trigger when set to "Last Position" after a few seconds.
    * Test your receiver's behavior! You can do this with the example sketch in the "Bolder Flight Systems SBUS" library.
    */
    int8_t i;

    //If using SBUS, this will only check the boolean failsafe value from the SBUS library.  If using PWM RX, it checks each channel.
    if (_type == sBUS) {
        if (_sbusFailSafe) {
            for (i=0; i<_num_channels; i++) {
                channel_pwm[i] = _channel_fs[i];
            }
        }
    }
    
    else {
        unsigned minVal = 800;
        unsigned maxVal = 2200;
        int check[6] = {0};
        int sum = 0;

        //Triggers for failure criteria -- if the PWM value < minvalue or > maxvalue, set a flag
        for (i=0; i<6; i++) {
            if (channel_pwm[i] > maxVal || channel_pwm[i] < minVal) check[i] = 1;
        }
        
        // Add up all the flags (if not failsafe, this will be 0)
        for (i=0; i<6; i++) {
            sum += check[i];
        }

        // If sum of all checks > 0, enable failsafe
        if (sum > 0) {
            for (i=0; i<_num_channels; i++) {
                channel_pwm[i] = _channel_fs[i];
            }
        }
    }
}


