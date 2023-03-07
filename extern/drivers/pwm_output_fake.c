#include "pwm_output_fake.h"

#include <stdbool.h>
/*
int16_t motorsPwm[MAX_SUPPORTED_MOTORS];

// PWM part
static bool pwmMotorsEnabled = false;
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;
*/
/*
void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse,
                  uint8_t motorCount) {
    UNUSED(motorConfig);
    UNUSED(motorCount);

    idlePulse = _idlePulse;
    if (motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) {
        idlePulse = 1000.0;
    }

    for (int motorIndex = 0;
         motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount;
         motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    pwmMotorsEnabled = true;
}
*/

/*
void servoDevInit(const servoDevConfig_t *servoConfig) {
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS;
         servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}
*/

/*
pwmOutputPort_t *pwmGetMotors(void) {
    return motors;
}
*/
/*
void pwmEnableMotors(void) {
    pwmMotorsEnabled = true;
}

bool pwmAreMotorsEnabled(void) {
    return pwmMotorsEnabled;
}
*/

/*
bool isMotorProtocolDshot(void) {
    return false;
}
*/
/*
void pwmWriteMotor(uint8_t index, float value) {
    motorsPwm[index] = (int16_t)(value - idlePulse);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount) {
    UNUSED(motorCount);
    pwmMotorsEnabled = false;
}
*/
/*
void pwmWriteServo(uint8_t index, float value) {
    servosPwm[index] = (int16_t)(value);
}
*/
/*
void pwmCompleteMotorUpdate(uint8_t motorCount) {
    UNUSED(motorCount);
}
*/