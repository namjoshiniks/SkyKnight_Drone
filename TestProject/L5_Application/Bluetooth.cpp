/*
 * Bluetooth.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: Admin
 */

#include "Bluetooth.hpp"
Uart3 *u3 = &(Uart3::getInstance());



void uartInit()
{
	u3->init(38400, 1000, 1000);
	u3->flush();
}

void setupBT()
{
	int i = 0;
	delay_ms(3000);
	while(1)
	{
		i++;
		if(i >= 3)
		{
			i = 0;
			break;
		}

	}
}

void BT(bool &BlueToothStartStopFlag,bool &BlueToothStopFlag, int &BluetoothDirectionFlag,bool &ThrottleIncrease,bool &ThrottleDecrease,bool& restoreValueFlag)
{
	char c;
	while(u3->getChar(&c, 100))
	{
		if(c == 'R')
		{
			BlueToothStartStopFlag = true;


		}
		else if(c == 'S')
		{
			BlueToothStopFlag = true;
		}
		else if (c == 'F')
		{
			BluetoothDirectionFlag = 3;

		}
		else if (c == 'Y')
		{
			BluetoothDirectionFlag = 4;
		}
		else if (c == 'L')
		{
			BluetoothDirectionFlag = 5;
		}
		else if (c == 'B')
		{
			BluetoothDirectionFlag = 6;
		}
		else if (c == 'G' )
		{
			BluetoothDirectionFlag = 0;
		}
		else if( c == 'D')
		{

			ThrottleDecrease = true;
			ThrottleIncrease = false;

		}
		else if( c == 'I')
		{
			ThrottleIncrease = true;
			ThrottleDecrease = false;
		}
		else if( c == 'K')
		{
			BlueToothStartStopFlag = false;
			if(BlueToothStopFlag)
			{
				BlueToothStopFlag = false;
				restoreValueFlag = true;
			}

		}
		else
		{
			BluetoothDirectionFlag = 0;
			printf(" ELSE \n");

		}

	}

}



