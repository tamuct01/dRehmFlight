

// Actuators.cpp
// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.



#include "Actuators.h"



Actuators::Actuators(OutputType type, uint8_t numActuators, uint8_t* actuatorPins) {
    _servos = nullptr;
    // _escs = nullptr;
    // _lsm6dsox = nullptr;
    uint8_t i;

    _type = type;
    _numActuators = numActuators;
    for (i=0; i<_numActuators; i++) {
        _actuatorPins[i] = actuatorPins[i];
    }

    if (_type == SERVO) {
        _servos = new PWMServo[_numActuators];
    }
    else if (_type == ESC) {
        _servos = new PWMServo[_numActuators];
    }
    else if (_type == ONESHOT125) {
        for (i=0; i<_numActuators; i++) {
            pinMode(_actuatorPins[i], OUTPUT);
        }
    }
    else {
        // error, unknown type
    }
}
    




  

bool Actuators::begin() {
    uint8_t i;

    switch (_type) {
        case SERVO:
        {
            for (i=0; i<_numActuators; i++) {
                _servos[i].attach(_actuatorPins[i], SERVO_PWM_MIN, SERVO_PWM_MAX); //Pin, min PWM value, max PWM value
                _servos[i].write(DEFAULT_SERVO_POS);
            }
            return true;
        }

        case ESC:
        {
            for (i=0; i<_numActuators; i++) {
                _servos[i].attach(_actuatorPins[i], SERVO_PWM_MIN, SERVO_PWM_MAX); //Pin, min PWM value, max PWM value
                _servos[i].write(DEFAULT_ESC_POS);
            }
            return true;
        }
        
        case ONESHOT125:
        {
            //Arm OneShot125 motors
            float output[_numActuators] = {DEFAULT_OS125_POS};

            //armMotors(); //Loop over commandMotors() until ESCs happily arm
            for (int i = 0; i <= 50; i++) {
                commandMotors(output);
                delay(2);
            }
        
            return true;
        }

        default:
            return false;
    }
}






void Actuators::commandOne(uint8_t index, float output) {
    if (index < _numActuators) {
        int angle;
        if (_type == SERVO || _type == ESC) {
            //Scaled to 0-180 for servo library
            angle = output * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[index].write(angle);
        }
        else if (_type == ONESHOT125) {

        }
        else {
            // Error
        }
    }
}

void Actuators::commandAll(float* output) {
    int angle;
    if (_type == SERVO || _type == ESC) {
        for (uint8_t i = 0; i < _numActuators; i++) {
            //Scaled to 0-180 for servo library
            angle = output[i] * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[i].write(angle);
        }
    }
    else if (_type == ONESHOT125) {

    }
    else {
        // Error
    }  
}

void Actuators::commandAllSame(float output) {
    int angle;
    if (_type == SERVO || _type == ESC) {
        for (uint8_t i = 0; i < _numActuators; i++) {
            //Scaled to 0-180 for servo library
            angle = output * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[i].write(angle);
        }
    }
    else if (_type == ONESHOT125) {

    }
    else {
        // Error
    }  
}



void Actuators::commandMotors(float* output) {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
    int wentLow = 0;
    int pulseStart, timer;
    int flagM[_numActuators] = {0};
    int OSoutput = 125;
    
    //Write all motor pins high
    for (uint8_t i = 0; i < _numActuators; i++) {
        digitalWrite(_actuatorPins[i], HIGH);
    }
    pulseStart = micros();

    //Write each motor pin low as correct pulse length is reached
    while (wentLow < 6 ) { //Keep going until final (6th) pulse is finished, then done
        timer = micros();
        for (uint8_t i = 0; i < _numActuators; i++) {
            //Scaled to 125us - 250us for oneshot125 protocol
            OSoutput = output[i]*125 + 125;
            //Constrain commands to motors within oneshot125 bounds
            OSoutput = constrain(OSoutput, 125, 250);

            if ((OSoutput <= timer - pulseStart) && (flagM[i]==0)) {
                digitalWrite(_actuatorPins[i], LOW);
                wentLow = wentLow + 1;
                flagM[i] = 1;
            }
        }
    }
}