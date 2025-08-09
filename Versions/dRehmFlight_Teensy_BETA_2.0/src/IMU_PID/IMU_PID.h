


// IMU_PID.h

#ifndef IMU_PID_h
#define IMU_PID_h

#include <Arduino.h>
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication

// Include all possible sensor libraries
#include "../MPU6050/MPU6050.h"
#include "../MPU9250/MPU9250.h"
#include <Adafruit_LSM6DSOX.h>

enum IMUType {
    MPU6050_I2C,
    MPU9250_SPI,
    LSM6DSOX_SPI
};

enum Gyro_DPS {
	GYRO_250DPS,	// DEFAULT
	GYRO_500DPS,
	GYRO_1000DPS,
	GYRO_2000DPS
};

enum Accel_G {
	ACCEL_2G,		// DEFAULT
	ACCEL_4G,
	ACCEL_8G,
	ACCEL_16G
};

// Conversion factors for the LSM6DSOX
// degrees to radians and back
#define DEG2RAD  0.0174533f
#define RAD2DEG  57.29575496f
// "G factor" to meters/second and back
#define G2MS     9.80665f
#define MS2G     0.10197162129779283f




class IMU {
public:
	
	IMU(IMUType type);
	IMU(IMUType type, Gyro_DPS GyroDPS, Accel_G AccelG);
	bool begin();

	void getIMUdata();
	void calculate_IMU_error();
	void calibrateMagnetometer();

	void controlANGLE(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle);
	void controlANGLE2(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle);
	void controlRATE(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle);

	void Madgwick(float invSampleFreq);
	void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

	void setMagError();
	void setMagScale();
	void setAccError();
	void setGryoError();

	void setRollAnglePID(float Kp, float Ki, float Kd);
	void setPitchAnglePID(float Kp, float Ki, float Kd);
	
	void setRollRatePID(float Kp, float Ki, float Kd);
	void setPitchRatePID(float Kp, float Ki, float Kd);
	void setYawRatePID(float Kp, float Ki, float Kd);

	float getGyro(char dir);
	float getAccel(char dir);
	float getMag(char dir);
	
	float AccX, AccY, AccZ;
	float GyroX;
	float GyroY;
	float GyroZ;
	float MagX, MagY, MagZ;
	float roll_IMU, pitch_IMU, yaw_IMU;

	float roll_PID = 0;
	float pitch_PID = 0;
	float yaw_PID = 0;



private:
	//IMU:
	IMUType  _type;
	Gyro_DPS _gyroDPS;
	Accel_G  _accelG;

    MPU6050*  _mpu6050;
    MPU9250*  _mpu9250;
    Adafruit_LSM6DSOX* _lsm6dsox;

	float _gyro_scale_factor = 131.0;
	float _accel_scale_factor = 16384.0;

	// Used for LSM6DSOX
	sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;  //unused, but could be interesting

	
	float invSqrt(float x);

	float AccX_prev, AccY_prev, AccZ_prev;
	float GyroX_prev, GyroY_prev, GyroZ_prev;
	float MagX_prev, MagY_prev, MagZ_prev;
	float roll_IMU_prev, pitch_IMU_prev;

	//Controller:
	float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll;
	float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch;
	float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw;


	float q0 = 1.0f; //Initialize quaternion for madgwick filter
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;

	//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
	float B_madgwick = 0.04;  //Madgwick filter parameter
	float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
	float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
	float B_mag = 1.0;        //Magnetometer LP filter parameter

	//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
	float MagErrorX = 0.0;
	float MagErrorY = 0.0; 
	float MagErrorZ = 0.0;
	float MagScaleX = 1.0;
	float MagScaleY = 1.0;
	float MagScaleZ = 1.0;

	//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
	float AccErrorX = -0.01;
	float AccErrorY = 0.01;
	float AccErrorZ = 0.04;
	float GyroErrorX = 0.41;
	float GyroErrorY = 0.17;
	float GyroErrorZ = -0.67;


	// PIDs
	float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
	// float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
	// float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
	// float maxYaw = 160.0;     //Max yaw rate in deg/sec

	float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
	float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
	float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
	float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
	float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
	float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
	float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
	float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

	float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
	float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
	float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
	float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
	float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
	float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

	float Kp_yaw = 0.3;           //Yaw P-gain
	float Ki_yaw = 0.05;          //Yaw I-gain
	float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)



};











#endif