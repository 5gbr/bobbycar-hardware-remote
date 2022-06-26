#include <Arduino.h>

#include <cmath>

//mpu6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define ANALOG_LEFT_X 0U
#define ANALOG_LEFT_Y 1U
#define ANALOG_LEFT_BUTTON 2U

#define ANALOG_RIGHT_X 3U
#define ANALOG_RIGHT_Y 4U
#define ANALOG_RIGHT_BUTTON 5U

#define THROTTLE_RIGHT 6U
#define THROTTLE_LEFT 7U

#define WHEEL_FRONT_LEFT 0U
#define WHEEL_FRONT_RIGHT 1U
#define WHEEL_BACK_LEFT 2U
#define WHEEL_BACK_RIGHT 3U

#define STICK_MODE_BOTH 0U
#define STICK_MODE_LEFT 1U
#define STICK_MODE_RIGHT 2U
#define STICK_MODE_THROTTLES 3U

// orientation/motion vars MPU
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU()
{
     // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin(16, 17);   // sda, scl;
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties



    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


class Inputs
{
public:
    void setPWMs(int front_steer_pwm, int back_steer_pwm, int front_drive_pwm, int back_drive_pwm)
    {
        _front_steer_pwm = front_steer_pwm;
        _back_steer_pwm = back_steer_pwm;
        _front_drive_pwm = front_drive_pwm;
        _back_drive_pwm = back_drive_pwm;
    }

    int getWheelValue(int wheel)
    {
        auto x = getX();
        auto y = getY();
        switch (wheel)
        {
        case WHEEL_FRONT_LEFT:
            return (x * _front_steer_pwm) + (y * _front_drive_pwm);
            break;

        case WHEEL_FRONT_RIGHT:
            return (-x * _front_steer_pwm) + (y * _front_drive_pwm);
            break;

        case WHEEL_BACK_LEFT:
            return (x * _back_steer_pwm) + (y * _back_drive_pwm);
            break;

        case WHEEL_BACK_RIGHT:
            return (-x * _back_steer_pwm) + (y * _back_drive_pwm);
            break;

        default:
            return 0;
            break;
        }
    }

    void init()
    {
        pinMode(LEFT_X_PIN, INPUT);
        pinMode(LEFT_Y_PIN, INPUT);
        pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);

        pinMode(RIGHT_X_PIN, INPUT);
        pinMode(RIGHT_Y_PIN, INPUT);
        pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
    }

    int getAxisValue(uint function)
    {
        switch (function)
        {
        case ANALOG_LEFT_X:
            return LEFT_X_VAL;
            break;

        case ANALOG_LEFT_Y:
            return LEFT_Y_VAL;
            break;

        case ANALOG_RIGHT_X:
            return RIGHT_X_VAL;
            break;

        case ANALOG_RIGHT_Y:
            return RIGHT_Y_VAL;
            break;
        default:
            return 0;
            break;
        }
    }

    bool getButtonValue(uint function)
    {
        switch (function)
        {
        case ANALOG_LEFT_BUTTON:
            return LEFT_BUTTON_VAL;
            break;
        case ANALOG_RIGHT_BUTTON:
            return RIGHT_BUTTON_VAL;
            break;
        default:
            return false;
            break;
        }
    }

    void setPin(uint function, uint pin)
    {
        switch (function)
        {
        case ANALOG_LEFT_X:
            LEFT_X_PIN = pin;
            break;
        case ANALOG_LEFT_Y:
            LEFT_Y_PIN = pin;
            break;
        case ANALOG_LEFT_BUTTON:
            LEFT_BUTTON_PIN = pin;
            break;

        case ANALOG_RIGHT_X:
            RIGHT_X_PIN = pin;
            break;
        case ANALOG_RIGHT_Y:
            RIGHT_Y_PIN = pin;
            break;
        case ANALOG_RIGHT_BUTTON:
            RIGHT_BUTTON_PIN = pin;
            break;

        case THROTTLE_LEFT:
            THROTTLE_LEFT_PIN = pin;
            break;
        case THROTTLE_RIGHT:
            THROTTLE_RIGHT_PIN = pin;
            break;

        default:
            assert(0);
        }
    }

    void update()
    {
        auto raw_lx = analogRead(LEFT_X_PIN);
        auto raw_ly = analogRead(LEFT_Y_PIN);
        auto raw_rx = analogRead(RIGHT_X_PIN);
        auto raw_ry = analogRead(RIGHT_Y_PIN);

        auto raw_tl = analogRead(THROTTLE_LEFT_PIN);
        auto raw_tr = analogRead(THROTTLE_RIGHT_PIN);

        auto raw_gy = readGyroYaw();

        LEFT_X_VAL = mapAnalogStick(lXMiddle, lXStart, lXEnd, raw_lx);
        LEFT_Y_VAL = mapAnalogStick(lYMiddle, lYStart, lYEnd, raw_ly);
        RIGHT_X_VAL = mapAnalogStick(rXMiddle, rXStart, rXEnd, raw_rx);
        RIGHT_Y_VAL = mapAnalogStick(rYMiddle, rYStart, rYEnd, raw_ry);

        THROTTLE_LEFT_VAL = mapThrottle(ltStart, ltEnd, raw_tl);
        THROTTLE_RIGHT_VAL = mapThrottle(rtStart, rtEnd, raw_tr);

        LEFT_BUTTON_VAL = !digitalRead(LEFT_BUTTON_PIN);
        RIGHT_BUTTON_VAL = !digitalRead(RIGHT_BUTTON_PIN);

        WHEEL_GYRO_VAL = mapWheelGyro(60, raw_gy);
        //Serial.printf("wheel=%d\n", WHEEL_GYRO_VAL);
    }

    void setCalibrationValues(int leftXMiddle, int leftXStart, int leftXEnd, int leftYMiddle, int leftYStart, int leftYEnd, int rightXMiddle, int rightXStart, int rightXEnd, int rightYMiddle, int rightYStart, int rightYEnd, int leftTStart, int leftTEnd, int rightTStart, int rightTEnd)
    {
        lXMiddle = leftXMiddle;
        lXStart = leftXStart;
        lXEnd = leftXEnd;

        lYMiddle = leftYMiddle;
        lYStart = leftYStart;
        lYEnd = leftYEnd;

        rXMiddle = rightXMiddle;
        rXStart = rightXStart;
        rXEnd = rightXEnd;

        rYMiddle = rightYMiddle;
        rYStart = rightYStart;
        rYEnd = rightYEnd;

        ltEnd = leftTEnd;
        rtEnd = rightTEnd;
        ltStart = leftTStart;
        rtStart = rightTStart;

        Serial.printf("ltstart=%d, ltend=%d", ltStart, ltEnd);
        Serial.printf("rtstart=%d, rtend=%d", rtStart, rtEnd);
    }

    void setDrivingMode(int DRIVING_MODE) {
        if (DRIVING_MODE == STICK_MODE_LEFT || DRIVING_MODE == STICK_MODE_RIGHT || DRIVING_MODE == STICK_MODE_BOTH || DRIVING_MODE == STICK_MODE_THROTTLES) {
            analog_stick_mode = DRIVING_MODE;
        } else {
            analog_stick_mode = STICK_MODE_LEFT;
        }
        NVS.setInt("driveMode", analog_stick_mode);
    }

    int getDrivingMode() {
        return analog_stick_mode;
    }
    int readGyroYaw()
    {
        // if programming failed, don't try to do anything
        if (!dmpReady) return 200;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
           // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        return ((int) (ypr[0] * 180/M_PI));
    }
private:
    int lXMiddle = LEFT_ANALOG_X_MIDDLE;
    int lXStart = LEFT_ANALOG_X_START;
    int lXEnd = LEFT_ANALOG_X_END;

    int lYMiddle = LEFT_ANALOG_Y_MIDDLE;
    int lYStart = LEFT_ANALOG_Y_START;
    int lYEnd = LEFT_ANALOG_Y_END;

    int rXMiddle = RIGHT_ANALOG_X_MIDDLE;
    int rXStart = RIGHT_ANALOG_X_START;
    int rXEnd = RIGHT_ANALOG_X_END;

    int rYMiddle = RIGHT_ANALOG_Y_MIDDLE;
    int rYStart = RIGHT_ANALOG_Y_START;
    int rYEnd = RIGHT_ANALOG_Y_END;

    int ltStart = LEFT_THROTTLE_START;
    int ltEnd = LEFT_THROTTLE_END;
    int rtStart = RIGHT_THROTTLE_START;
    int rtEnd = RIGHT_THROTTLE_END;


    float getX()
    {
        switch (analog_stick_mode)
        {
        case STICK_MODE_BOTH:
            return (RIGHT_Y_VAL / 100.f);
            break;

        case STICK_MODE_RIGHT:
            return (-RIGHT_Y_VAL / 100.f);
            break;

        case STICK_MODE_THROTTLES:
            return (WHEEL_GYRO_VAL / 100.f);
            break;

        default: // STICK_MODE_LEFT
            return (LEFT_X_VAL / 100.f);
            break;
        }
    }

    float getY()
    {
        switch (analog_stick_mode)
        {
        case STICK_MODE_BOTH:
            return (LEFT_Y_VAL / 100.f);
            break;

        case STICK_MODE_RIGHT:
            return (RIGHT_X_VAL / 100.f);
            break;

        case STICK_MODE_THROTTLES:
            return ((THROTTLE_RIGHT_VAL / 100.f) - (THROTTLE_LEFT_VAL / 100.f));
            break;
        
        default: // STICK_MODE_LEFT
            return (LEFT_Y_VAL / 100.f);
            break;
        }
    }

    int analog_stick_mode = STICK_MODE_LEFT;

    // PWMs
    int _front_steer_pwm = 100;
    int _back_steer_pwm = 0;
    int _front_drive_pwm = 75;
    int _back_drive_pwm = 100;

    // Left Analog stick
    // X
    int LEFT_X_PIN;
    int LEFT_X_VAL;
    // Y
    int LEFT_Y_PIN;
    int LEFT_Y_VAL;
    // Button
    int LEFT_BUTTON_PIN;
    bool LEFT_BUTTON_VAL;

    // Right Analog stick
    // X
    int RIGHT_X_PIN;
    int RIGHT_X_VAL;
    // Y
    int RIGHT_Y_PIN;
    int RIGHT_Y_VAL;
    // Button
    int RIGHT_BUTTON_PIN;
    bool RIGHT_BUTTON_VAL;
    //Throttle
    int THROTTLE_RIGHT_PIN;
    int THROTTLE_LEFT_PIN;
    int THROTTLE_LEFT_VAL;
    int THROTTLE_RIGHT_VAL;
    //Wheel
    int WHEEL_GYRO_VAL;

    int mapAnalogStick(int middle, int start, int end, int raw)
    {
        if (abs(raw - middle) < DEADBAND)
        {
            return 0;
        }
        else if (raw < middle)
        {
            auto return_val = map(raw, start, middle - DEADBAND, -100, 0);
            if (return_val > 0)
                return 0;
            if (return_val < -100)
                return -100;
            return return_val;
        }
        else
        {
            auto return_val = map(raw, middle + DEADBAND, end, 0, 100);
            if (return_val < 0)
                return 0;
            if (return_val > 100)
                return 100;
            return return_val;
        }
    }

    int mapThrottle(int start, int end, uint16_t traw)
    {
        int throttle_diff,tmin,tmax;
        if (end < start)
        {
            throttle_diff = start - traw - THROTTLE_DEADBAND;
        }
        else
        {
            //TODO
        }
        if (throttle_diff < 0)
            return 0;
        else
        {
            auto return_val = map(throttle_diff, 0, abs(end-start), 0, 100);
            if (return_val < 0)
                return 0;
            if (return_val > 100)
                return 100;
            return return_val;
        }
    }

    int mapWheelGyro(int range, int yraw)
    {
        int retval = map(yraw, -range, range, -100, 100);
        if (retval < 100)
        {
            if (retval > -100)
                return retval;
            else
                return -100;
        }
        else
            return 100;
        
    }
};