#include "ArduinoDevice.h"
#include <stdint.h>
#include <tchar.h>
#include <Windows.h>

#define fosc 16000000UL
#define UBRRn 0

enum Opcode
{
    Nothing,

    Enum_DC_Motor_Initialise,
    Enum_DC_Motor_Control_CW_CW,
    Enum_DC_Motor_Control_CW_CCW,
    Enum_DC_Motor_Control_CCW_CW,
    Enum_DC_Motor_Control_CCW_CCW,
    Enum_DC_Motor_Brake,
    Enum_DC_Motor_Current_Sensing,

    Enum_ServoStart,
    Enum_ServoMicrosecond,
    Enum_ServoWait,

    Enum_PinLevel,

    Enum_MR2x30a_Start,
    Enum_MR2x30a_Speed,
    Enum_MR2x30a_Brake,

    // Need to add
    //ArduinoOpen
    //ArduinoClose
    //ArduinoServoWait
    //TimeOut
    //AnalogSignal
    //DigitalSignal
};

typedef struct Arduino
{
    HANDLE hFile;
    OVERLAPPED Read;
    OVERLAPPED Write;
}Arduino;

Arduino* ArduinoOpen(LPCTSTR lpFileName)
{
    HANDLE hFile = CreateFile
    (
        lpFileName,
        GENERIC_READ | GENERIC_WRITE,
        0UL,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL
    );
    if (INVALID_HANDLE_VALUE == hFile)
    {
        return NULL;
    }

    DCB dcb =
    {
        .DCBlength = sizeof(DCB),
        .BaudRate = fosc / (8 * (UBRRn + 1)),
        .ByteSize = 8,
        .Parity = NOPARITY,
        .StopBits = ONESTOPBIT,
        .fDtrControl = DTR_CONTROL_DISABLE
    };

    if (FALSE == SetCommState(hFile, &dcb))
    {
        return NULL;
    }

    HANDLE Read = CreateEvent(NULL, TRUE, TRUE, NULL);
    if (NULL == Read)
    {
        return NULL;
    }

    HANDLE Write = CreateEvent(NULL, TRUE, TRUE, NULL);
    if (NULL == Write)
    {
        return NULL;
    }

    Arduino *pArduino = (Arduino*)malloc(sizeof(Arduino));
    if (NULL == pArduino)
    {
        return NULL;
    }

    memset(pArduino, 0, sizeof(Arduino));
    pArduino->hFile = hFile;
    pArduino->Read.hEvent = Read;
    pArduino->Write.hEvent = Write;
    return pArduino;
}

BOOL ArduinoClose(Arduino* pArduino)
{
#ifdef _MSC_VER
    DWORD dResult = WaitForSingleObject(pArduino->Read.hEvent, INFINITE);
    if (WAIT_OBJECT_0 != dResult)
    {
        return FALSE;
    }

    dResult = WaitForSingleObject(pArduino->Write.hEvent, INFINITE);
    if (WAIT_OBJECT_0 != dResult)
    {
        return FALSE;
    }
#endif // _MSC_VER

    BOOL bResult = CloseHandle(pArduino->Read.hEvent);
    if (FALSE == bResult)
    {
        return FALSE;
    }

    bResult = CloseHandle(pArduino->Write.hEvent);
    if (FALSE == bResult)
    {
        return FALSE;
    }

    bResult = CloseHandle(pArduino->hFile);
    if (FALSE == bResult)
    {
        return FALSE;
    }

    return TRUE;
}

BOOL ArduinoRead(Arduino* pArduino, LPVOID lpBuffer, DWORD Byte)
{
    DWORD NumberOfBytesRead = 0UL;
    ReadFile(pArduino->hFile, lpBuffer, Byte, &NumberOfBytesRead, &pArduino->Read);

    DWORD Error = GetLastError();
    if (ERROR_IO_PENDING != Error)
    {
        return FALSE;
    }

    DWORD Result = WaitForSingleObject(pArduino->Read.hEvent, INFINITE);
    if (WAIT_OBJECT_0 != Result)
    {
        return FALSE;
    }

    return TRUE;
}

BOOL ArduinoWrite(Arduino* pArduino, LPVOID lpBuffer, DWORD Byte)
{
    DWORD Result = WaitForSingleObject(pArduino->Write.hEvent, INFINITE);
    if (WAIT_OBJECT_0 != Result)
    {
        return FALSE;
    }

    DWORD NumberOfBytesWritten = 0UL;
    WriteFile(pArduino->hFile, lpBuffer, Byte, &NumberOfBytesWritten, &pArduino->Write);

    DWORD Error = GetLastError();
    if (ERROR_IO_PENDING != Error)
    {
        return FALSE;
    }

    return TRUE;
}

BOOL ArduinoMotorSpeed(Arduino* pArduino, int Left, int Right)
{
    enum Opcode Opcode;

    if (0 < Left)
    {
        if (0 < Right)
        {
            Opcode = Enum_DC_Motor_Control_CW_CW;
        }
        else // 0 >= Right
        {
            Right = -Right;
            Opcode = Enum_DC_Motor_Control_CW_CCW;
        }
    }
    else // 0 >= Left
    {
        Left = -Left;
        if (0 < Right)
        {
            Opcode = Enum_DC_Motor_Control_CCW_CW;
        }
        else // 0 >= Right
        {
            Right = -Right;
            Opcode = Enum_DC_Motor_Control_CCW_CCW;
        }
    }

#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
        uint8_t Left;
        uint8_t Right;
    }Data = { Opcode, (uint8_t)Left, (uint8_t)Right };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

BOOL ArduinoMotorBrake(Arduino* pArduino)
{
#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
    }
    Data = { Enum_DC_Motor_Brake };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

BOOL ArduinoMR2x30aSpeed(Arduino* pArduino, int Left, int Right)
{
#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
        int16_t Left;
        int16_t Right;
    }
    Data = { Enum_MR2x30a_Speed, (int16_t)Left, (int16_t)Right };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

BOOL ArduinoMR2x30aBrake(Arduino* pArduino)
{
#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
    }
    Data = { Enum_MR2x30a_Brake };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

BOOL ArduinoServoMicrosecond(Arduino* pArduino, int ServoIndex, int SetMicroseconds, int SpeedMicroseconds)
{
#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
        uint8_t ServoIndex;
        uint16_t SetMicroseconds;
        uint16_t SpeedMicroseconds;
    }
    Data = { Enum_ServoMicrosecond, (uint8_t)ServoIndex, (uint16_t)SetMicroseconds, (uint16_t)SpeedMicroseconds };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

BOOL ArduinoServoWait(Arduino* pArduino, int ServoIndex)
{
#pragma pack(push, 1)
    struct
    {
        uint8_t Opcode;
        uint8_t ServoIndex;
    }
    Data = { Enum_ServoWait, (uint8_t)ServoIndex };
#pragma pack(pop)

    return ArduinoWrite(pArduino, &Data, sizeof(Data));
}

Arduino* arduino_open(const char* pFileName)
{
    // Warning C4133
    return ArduinoOpen(pFileName);
}

bool arduino_close(Arduino* pArduino)
{
    return ArduinoClose(pArduino);
}

bool arduino_read(Arduino* pArduino, void* pBuffer, int Byte)
{
    return ArduinoRead(pArduino, pBuffer, (DWORD)Byte);
}

bool arduino_write(Arduino* pArduino, void* pBuffer, int Byte)
{
    return ArduinoWrite(pArduino, pBuffer, (DWORD)Byte);
}

bool arduino_motor_speed(Arduino* pArduino, int Left, int Right)
{
    return ArduinoMotorSpeed(pArduino, Left, Right);
}

bool arduino_motor_brake(Arduino* pArduino)
{
    return ArduinoMotorBrake(pArduino);
}

bool arduino_mr2x30a_speed(Arduino* pArduino, int Left, int Right)
{
    return ArduinoMR2x30aSpeed(pArduino, Left, Right);
}

bool arduino_mr2x30a_brake(Arduino* pArduino)
{
    return ArduinoMR2x30aBrake(pArduino);
}

bool arduino_servo_microsecond(Arduino* pArduino, int ServoIndex, int SetMicroseconds, int SpeedMicroseconds)
{
    return ArduinoServoMicrosecond(pArduino, ServoIndex, SetMicroseconds, SpeedMicroseconds);
}

bool arduino_servo_wait(Arduino* pArduino, int ServoIndex)
{
    return ArduinoServoWait(pArduino, ServoIndex);
}
