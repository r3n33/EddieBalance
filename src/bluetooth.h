#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdlib.h>

void BluetoothInit()
{
	system( "rfkill unblock bluetooth" );
}

#endif //--BLUETOOTH_H