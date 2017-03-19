#pragma once
#include <stdbool.h>

#define COMM_PORT_NAME(Port) ("\\\\.\\COM"#Port)

#ifdef __cplusplus 
extern "C" {
#endif

    typedef struct Arduino Arduino;

    Arduino* arduino_open(const char* pFileName);
    bool arduino_close(Arduino* pArduino);

    bool arduino_read(Arduino* pArduino, void* pBuffer, int Byte);
    bool arduino_write(Arduino* pArduino, void* pBuffer, int Byte);

    bool arduino_motor_speed(Arduino* pArduino, int Left, int Right);
    bool arduino_motor_brake(Arduino* pArduino);

    bool arduino_mr2x30a_speed(Arduino* pArduino, int left, int right);
    bool arduino_mr2x30a_brake(Arduino* pArduino);

    bool arduino_servo_microsecond(Arduino* pArduino, int ServoIndex, int SetMicroseconds, int SpeedMicroseconds);
    bool arduino_servo_wait(Arduino* pArduino, int ServoIndex);

#ifdef _WINDOWS_

#ifdef __cplusplus 
}
#endif

#include <tchar.h>

#define CommPortName(Port) (_T("\\\\.\\COM") ## _T(#Port))

#ifdef __cplusplus 
extern "C" {
#endif

    Arduino* ArduinoOpen(LPCTSTR lpFileName);
    BOOL ArduinoClose(Arduino* pArduino);

    BOOL ArduinoRead(Arduino* pArduino, LPVOID lpBuffer, DWORD Byte);
    BOOL ArduinoWrite(Arduino* pArduino, LPVOID lpBuffer, DWORD Byte);

    BOOL ArduinoMotorSpeed(Arduino* pArduino, int Left, int Right);
    BOOL ArduinoMotorBrake(Arduino* pArduino);

    BOOL ArduinoMR2x30aSpeed(Arduino* pArduino, int Left, int Right);
    BOOL ArduinoMR2x30aBrake(Arduino* pArduino);

    BOOL ArduinoServoMicrosecond(Arduino* pArduino, int ServoIndex, int SetMicroseconds, int SpeedMicroseconds);
    BOOL ArduinoServoWait(Arduino* pArduino, int ServoIndex);

#ifdef __cplusplus 
}
#endif

#endif
