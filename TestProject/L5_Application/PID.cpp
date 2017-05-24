/*
 * PID.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: Admin
 */
#include "PID.hpp"

float FR_PWM_MAX = baseVal + 10;
float FR_PWM_MIN = baseVal - 10;


/* Pitch variables */
float error_pitch = 0, lasterror_pitch = 0, set_pitch = 0;
float errorIntegralPitch[6] = {0};
float sumIntegralPitch = 0;

/* Roll variables */
float error_roll = 0, lasterror_roll = 0, set_roll = 0;
float errorIntegralRoll[6] = {0};
float sumIntegralRoll = 0;

/* Yaw variables */
float error_yaw = 0, lasterror_yaw = 0, set_yaw = 0;
float errorIntegralYaw[6] = {0};
float sumIntegralYaw = 0;

float FR_TELEM = 0;
float FL_TELEM = 0;


void PID_Algorithm_Pitch(bool &BlueToothStartStopFlag,bool &BlueToothStopFlag,int &BluetoothDirectionFlag,bool &ThrottleIncrease,bool &ThrottleDecrease,bool& restoreValueFlag, uint32_t &count,float &pitchAngle,float &rollAngle,float &yawAngle)
{
	static PWM FR(PWM::pwm1,400.0);
	static PWM FL(PWM::pwm2,400.0);
	static PWM BR(PWM::pwm3,400.0);
	static PWM BL(PWM::pwm4,400.0);

	if(count < 300)
	{
		FR.set(40);
		FL.set(40);
		BR.set(40);
		BL.set(40);
	}
	else
	{
		if(BlueToothStartStopFlag)
		{
			PID_Function(BluetoothDirectionFlag,BlueToothStopFlag,ThrottleIncrease,ThrottleDecrease);

			FR_DCPWM = baseVal;
			FL_DCPWM = baseVal;
			BR_DCPWM = baseVal;
			BL_DCPWM = baseVal;
			//printf("BaseVal %0.2f\n",baseVal);




			error_pitch = (set_pitch - (pitchAngle));

			P_Pitch = error_pitch;
			//printf("Pitch Angle = %0.2f \n", pitchAngle);
			for(int i = 0 ; i < 5 ; i++)
			{
				errorIntegralPitch[i] = errorIntegralPitch[i+1];
			}
			errorIntegralPitch[5] = error_pitch;
			sumIntegralPitch = 0;
			for(int i = 0; i < 6; i++)
			{
				sumIntegralPitch += abs(errorIntegralPitch[i]);
			}

			I_Pitch = (sumIntegralPitch * 0.01);

			D_Pitch = (error_pitch - lasterror_pitch)/0.01;

			float n = P_Pitch * 0.7 + D_Pitch * 0.07 + I_Pitch * 1;
			//printf("Error_Pitch %0.2f \n",error_pitch);
			//		printf("LastError_Pitch %0.2f \n",lasterror_pitch);
			//		printf("D Pitch %0.2f\n",D_Pitch* 0.08);
			//		printf("P Pitch %0.2f\n",P_Pitch * 2);

			//printf("P n = %0.2f\n",P_Pitch * 0.05);
			//		printf("Actual n = %0.2f\n",n);

			n = 0.11 * n;
			//		printf("PWM = %0.2f\n",n);

			lasterror_pitch = error_pitch;
			FR_DCPWM -= n;
			FL_DCPWM -= n;
			BR_DCPWM += n;
			BL_DCPWM += n;

			//PWM_Limit();
			//		printf("FR_DCPWM = %0.2f\n", FR_DCPWM);
			//		printf("FL_DCPWM  = %0.2f\n", FL_DCPWM);


			//		printf("FR_DCPWM = %0.2f\n", FR_DCPWM);
			//		printf("FL_DCPWM  = %0.2f\n", FL_DCPWM);
			//		printf("BR_DCPWM = %0.2f\n", BR_DCPWM);
			//		printf("BL_DCPWM  = %0.2f\n", BL_DCPWM);
			//		printf("Addition  = %0.2f\n", FR_DCPWM + FL_DCPWM + BR_DCPWM + BL_DCPWM);

			//		//PID_Function(BlueToothOrder,FR,FL,BR,BL);
		}
		else
		{

			if(restoreValueFlag)
			{
				baseVal = initialBaseVal;
				restoreValueFlag = false;
			}
			FR.set(40);
			FL.set(40);
			BR.set(40);
			BL.set(40);
			FR_PWM_MAX = baseVal + 10;
			FR_PWM_MIN = baseVal - 10;
			//printf("Stop Pitch\n");
			lasterror_pitch = 0;
			for(int i = 0; i < 6; i++)
			{
				errorIntegralPitch[i] = 0;
			}
			set_pitch = 0;
		}
	}
}

void PID_Algorithm_Roll(bool &BlueToothStartStopFlag,bool &BlueToothStopFlag,int &BluetoothDirectionFlag,bool &ThrottleIncrease,bool &ThrottleDecrease, uint32_t &count,float &pitchAngle,float &rollAngle,float &yawAngle)
{
	static PWM FR(PWM::pwm1,400.0);
	static PWM FL(PWM::pwm2,400.0);
	static PWM BR(PWM::pwm3,400.0);
	static PWM BL(PWM::pwm4,400.0);

	if(count < 300)
	{
		FR.set(40);
		FL.set(40);
		BR.set(40);
		BL.set(40);
	}
	else
	{
		if(BlueToothStartStopFlag)
		{

			error_roll = (set_roll - (rollAngle));
			//printf("Roll Angle = %0.2f \n",rollAngle);

			P_Roll = error_roll;
			for(int i = 0 ; i < 5 ; i++)
			{
				errorIntegralRoll[i] = errorIntegralRoll[i+1];
			}
			errorIntegralRoll[5] = error_roll;
			sumIntegralRoll = 0;
			for(int i = 0; i < 6; i++)
			{
				sumIntegralRoll += abs(errorIntegralRoll[i]);
			}


			I_Roll = (sumIntegralRoll * 0.01);

			D_Roll = (error_roll - lasterror_roll)/0.01;


			float n = P_Roll * 0.7 + D_Roll * 0.07 + I_Roll * 1; //

			n = 0.11 * n;

			lasterror_roll = error_roll;
			FR_DCPWM -= n;
			FL_DCPWM += n;
			BR_DCPWM -= n;
			BL_DCPWM += n;





		}
		else
		{
			//printf("Stop Roll\n");
			FR.set(40);
			FL.set(40);
			BR.set(40);
			BL.set(40);
			lasterror_roll = 0;
			for(int i = 0; i < 6; i++)
			{
				errorIntegralRoll[i] = 0;
			}
			set_roll = 0;
		}
	}

}



void PID_Algorithm_Yaw(bool &BlueToothStartStopFlag,bool &BlueToothStopFlag,int &BluetoothDirectionFlag,bool &ThrottleIncrease,bool &ThrottleDecrease, uint32_t &count,float &pitchAngle,float &rollAngle,float &yawAngle)
{
	static PWM FR(PWM::pwm1,400.0);
	static PWM FL(PWM::pwm2,400.0);
	static PWM BR(PWM::pwm3,400.0);
	static PWM BL(PWM::pwm4,400.0);

	if(count < 300)
	{
		FR.set(40);
		FL.set(40);
		BR.set(40);
		BL.set(40);
	}
	else
	{
		if(BlueToothStartStopFlag)
		{
			error_yaw = (set_yaw - (yawAngle));
			//printf("Yaw Angle = %0.2f \n",yawAngle);

			P_Yaw = error_yaw;
			for(int i = 0 ; i < 5 ; i++)
			{
				errorIntegralYaw[i] = errorIntegralYaw[i+1];
			}
			errorIntegralYaw[5] = error_yaw;
			sumIntegralYaw = 0;
			for(int i = 0; i < 6; i++)
			{
				sumIntegralYaw += abs(errorIntegralYaw[i]);
			}


			I_Yaw = (sumIntegralYaw * 0.01);

			D_Yaw = (error_yaw - lasterror_yaw)/0.01;


			float n = P_Yaw * 0.01 + D_Yaw * 0.0009 ;//+ I_Yaw * 0.5;

			n = 0.11 * n;

			lasterror_yaw = error_yaw;
			FR_DCPWM -= n;
			FL_DCPWM += n;
			BR_DCPWM += n;
			BL_DCPWM -= n;
			//			printf("FR_DCPWM = %0.2f\n", FR_DCPWM);
			//			printf("FL_DCPWM  = %0.2f\n", FL_DCPWM);
			//			printf("BR_DCPWM = %0.2f\n", BR_DCPWM);
			//			printf("BL_DCPWM  = %0.2f\n", BL_DCPWM);
			FR_DCPWM += 0.6;
			FL_DCPWM -= 0.6;
			BR_DCPWM += 0.6;
			BL_DCPWM -= 0.6;

			PWM_Limit();

			FR.set(FR_DCPWM);
			FL.set(FL_DCPWM);
			BR.set(BR_DCPWM);
			BL.set(BL_DCPWM);

			LE.on(1);
			//printf("FR_DCPWM = %0.2f\n", FR_DCPWM);
			//printf("FL_DCPWM  = %0.2f\n", FL_DCPWM);
			//printf("BR_DCPWM = %0.2f\n", BR_DCPWM);
			//printf("BL_DCPWM  = %0.2f\n", BL_DCPWM);
			//printf("Set Pitch %0.2f \n",set_pitch);
			//printf("Set Roll %0.2f \n",set_roll);

			//		printf("Addition  = %0.2f\n\n", FR_DCPWM + FL_DCPWM + BR_DCPWM + BL_DCPWM);

			//		//PID_Function(BlueToothOrder,FR,FL,BR,BL);
		}
		else
		{
			//printf("Stop Yaw\n");
			FR.set(40);
			FL.set(40);
			BR.set(40);
			BL.set(40);
			lasterror_yaw = 0;
			for(int i = 0; i < 6; i++)
			{
				errorIntegralYaw[i] = 0;
			}
			set_yaw = 0;
		}
	}




}

void PID_Function(int &BluetoothDirectionFlag,bool &BlueToothStopFlag,bool &ThrottleIncrease,bool &ThrottleDecrease)
{


	if(BlueToothStopFlag == true)
	{
		baseVal -= 0.005;
		FR_PWM_MAX = baseVal + 10;
		FR_PWM_MIN = baseVal - 10;
		if(FR_PWM_MIN < 40)
		{
			FR_PWM_MIN = 40;
		}
	}
	if(ThrottleIncrease)
	{
		float tempbase = baseVal;;
		tempbase += 0.1;
		if(tempbase >=60 && tempbase <=70)
		{
			baseVal = tempbase;
			FR_PWM_MAX = baseVal + 10;
			FR_PWM_MIN = baseVal - 10;
		}

		ThrottleIncrease = false;
	}
	if(ThrottleDecrease)
	{
		float tempbase = baseVal;;
		tempbase -= 0.1;
		if(tempbase >=60 && tempbase <=70)
		{
			baseVal = tempbase;
			FR_PWM_MAX = baseVal + 10;
			FR_PWM_MIN = baseVal - 10;
		}

		ThrottleDecrease = false;

	}
	if (BluetoothDirectionFlag == 3)
	{
		set_pitch = 8;
		set_roll = 0;
		set_yaw = 0;  // Forward
		printf("Forward.....\n");
	}
	else if (BluetoothDirectionFlag == 4)
	{
		set_pitch = 0;
		set_roll = 5;
		set_yaw = 0;// Right

		printf("Right.....\n");
	}
	else if (BluetoothDirectionFlag == 5)
	{
		set_pitch = 0;
		set_roll = -8;
		set_yaw = 0; //Left
		printf("Left.....\n");
	}
	else if (BluetoothDirectionFlag == 6)
	{
		set_pitch = -5;
		set_roll = 0;
		set_yaw = 0; //Back
		printf("Back.....\n");
	}
	else
	{
		set_pitch = 0;
		set_roll = 0;
		set_yaw = 0;
	}
}

void PWM_Limit()
{
	if(FR_DCPWM > FR_PWM_MAX )
	{
		FR_DCPWM = FR_PWM_MAX;
	}
	if(FL_DCPWM > FR_PWM_MAX )
	{
		FL_DCPWM = FR_PWM_MAX;
	}
	if(BR_DCPWM > FR_PWM_MAX )
	{
		BR_DCPWM = FR_PWM_MAX;
	}
	if(BL_DCPWM > FR_PWM_MAX )
	{
		BL_DCPWM = FR_PWM_MAX;
	}
	if(FR_DCPWM < FR_PWM_MIN )
	{
		FR_DCPWM = FR_PWM_MIN;
	}
	if(FL_DCPWM < FR_PWM_MIN )
	{
		FL_DCPWM = FR_PWM_MIN;
	}
	if(BR_DCPWM < FR_PWM_MIN )
	{
		BR_DCPWM = FR_PWM_MIN;
	}
	if(BL_DCPWM <  FR_PWM_MIN )
	{
		BL_DCPWM = FR_PWM_MIN;
	}
}

void ESC_CALIB(uint32_t& count)
{
	static PWM FR(PWM::pwm1,400.0);
	static PWM FL(PWM::pwm2,400.0);
	static PWM BR(PWM::pwm3,400.0);
	static PWM BL(PWM::pwm4,400.0);
	if(count < 30)
	{
		FR.set(80);
		FL.set(80);
		BR.set(80);
		BL.set(80);
	}
	else if(count >= 30 && count < 60)
	{
		FR.set(40);
		FL.set(40);
		BR.set(40);
		BL.set(40);
	}

}

void MotorTest(uint32_t& count)
{
	static PWM FR(PWM::pwm1,400.0);
	static PWM FL(PWM::pwm2,400.0);
	static PWM BR(PWM::pwm3,400.0);
	static PWM BL(PWM::pwm4,400.0);
	if(count < 30)
	{
		FR.set(40);
		FL.set(40);
		BR.set(40);
		BL.set(40);
	}
	else
	{
		FR.set(65);
		FL.set(65);
		BL.set(65);
		BR.set(65);
	}
}
