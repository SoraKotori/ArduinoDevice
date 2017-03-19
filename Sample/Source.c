#include <Windows.h>
#include "ArduinoDevice.h"
#include <stdlib.h>

int main(void)
{
    Arduino *_pArduino = ArduinoOpen(CommPortName(3));
    if (NULL == _pArduino)
    {
        return EXIT_FAILURE;
    }

    int _LeftSpeed = 140;
    int _RightSpeed = 140;

    BOOL Result = ArduinoMR2x30aSpeed(_pArduino, _LeftSpeed, _RightSpeed);
	if (Result == FALSE)
	{
		return EXIT_FAILURE;
	}

	Sleep(1000);
	Result = ArduinoMR2x30aSpeed(_pArduino, 0, 0);
	if (Result == FALSE)
	{
		return EXIT_FAILURE;
	}

	Result = ArduinoClose(_pArduino);
	if (Result == FALSE)
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
