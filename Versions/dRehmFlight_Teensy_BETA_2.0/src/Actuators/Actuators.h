// Actuators.h
// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.


#ifndef Actuators_h
#define Actuators_h

#include <Arduino.h>
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer


enum OutputType {
    SERVO,
    ESC,
    ONESHOT125
};

#define SERVO_PWM_MIN   900
#define SERVO_PWM_MAX  2100

#define DEFAULT_SERVO_POS   90
#define DEFAULT_ESC_POS      0
#define DEFAULT_OS125_POS  0.0



class Actuators {
public:
	
	Actuators(OutputType type, uint8_t numActuators, uint8_t* actuatorPins);

	bool begin();

    // Expect all of our outputs to be 0.0 to 1.0 from the Mixer.  Will scale appropriately
    void commandOne(uint8_t index, float output);
    void commandAll(float* output);
    void commandAllSame(float output);
    void commandMotors(float* output);



private:

	OutputType  _type;

    PWMServo* _servos;

    
    // oneshot  _oneshot;

    uint8_t _numActuators = 1;
    uint8_t _actuatorPins[];
};




#endif