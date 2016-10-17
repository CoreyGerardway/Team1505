/*-----------------------------------------------------------------------------*/
/*		setSkipperMotors - is used to define which motors should not be affected
/*											 by the slewing function.
/*
/*											0 - Motor not affected
/*											1 - Motor is affected
/*-----------------------------------------------------------------------------*/
void setSkipperMotors(){
	motorSkip[0]= 0;		//intake2
	motorSkip[1]= 1;
	motorSkip[2]= 1;			//Skips motor in port 2
	motorSkip[3]= 0;		//intake front
	motorSkip[4]= 1;
	motorSkip[5]= 1;
	motorSkip[6]= 0;   //intake back
	motorSkip[7]= 1;
	motorSkip[8]= 1;
	motorSkip[9]= 0;  //spool has no slew
}

/*-----------------------------------------------------------------------------*/
/*		 setMotorSlew - This function allows you to individual set the slew rates
/*
/*-----------------------------------------------------------------------------*/

void setSlewRate(){
	int laucherFix = 8;
	motorSlew[0]= MOTOR_DEFAULT_SLEW_RATE;
	motorSlew[1]= MOTOR_DEFAULT_SLEW_RATE;
	motorSlew[2]= MOTOR_DEFAULT_SLEW_RATE-laucherFix;			//Skips motor in port 2
	motorSlew[3]= MOTOR_DEFAULT_SLEW_RATE-laucherFix;
	motorSlew[4]= 20;											//faster recovery for drive
	motorSlew[5]= 20;											//faster recovery for drive
	motorSlew[6]= MOTOR_DEFAULT_SLEW_RATE-laucherFix;
	motorSlew[7]= MOTOR_DEFAULT_SLEW_RATE-laucherFix;
	motorSlew[8]= MOTOR_DEFAULT_SLEW_RATE;
	motorSlew[9]= MOTOR_DEFAULT_SLEW_RATE;
}

int setMotor(int current, int request, int motorIndex){
	if( current != request )
	{
		// increasing motor value
		if( request > current )
		{
			current += motorSlew[motorIndex];
			// limit
			if( current > MOTOR_MAX_VALUE )
				current = MOTOR_MAX_VALUE;
		}
		// decreasing motor value
		if( request < current )
		{
			current -= motorSlew[motorIndex];
			// limit
			if( current < MOTOR_MIN_VALUE )
				current = MOTOR_MIN_VALUE;
		}
		// finally set motor
	}
	return current;
}


task MotorSlewRateTask()
{
	int motorTmp;
	// Initialize stuff
	setSlewRate(); //initialize the slew rates
	setSkipperMotors(); //initialize which motors can skipped
	for(int motorIndex=0;motorIndex<MOTOR_NUM;motorIndex++)
	{
		motorReq[motorIndex] = 0;
	}
	// run task until stopped
	while( true )
	{
		// run loop for every motor
		for(int motorIndex=0; motorIndex<MOTOR_NUM; motorIndex++)
		{
			if (motorSkip[motorIndex]){// So we don't keep accessing the internal storage
				motorTmp = motor[ motorIndex ];
				motor[motorIndex] = setMotor(motorTmp,motorReq[motorIndex], motorIndex );
				//	writeDebugStreamLine( "%d", motor[motorIndex]);
			}//end if motor is allowed to change
			else{
				motor[motorIndex] = motorReq[motorIndex];
			}
		}//end for loop
		// Wait approx the speed of motor update over the spi bus
		wait1Msec( MOTOR_TASK_DELAY );
	}
}


// END TASK SLEW


task recordAutoCode(){
	//int motorIndex;
	writeDebugStream("%d \n%d \n", HASH_SPEED, MOTOR_NUM);
	for(int j=0;j<MOTOR_NUM;j++) // i is motor index
	{
		writeDebugStreamLine("%s", MOTOR_NAME[j] );
	}
	writeDebugStream("sensors\n%d\n", SENSOR_NUM);
	for(int x=0;x<SENSOR_NUM;x++)  //x is sensor index
	{
		writeDebugStreamLine("%s", SENSOR_NAME[x]);
	}
	while(true){
		if (vexRT(STARTRECORD)){
			recodeMode = 1;
		}
		else if (vexRT(ENDRECORD)){
			recodeMode=0;
		}
		if (recodeMode == 1){
			//	writeDebugStream("motor[%d] = ", motorIndex);
			int sensor1 = SensorValue(enc1);
			//int sensor2 = SensorValue(enc2); //add more variables based on sensors
			writeDebugStream("m%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;K\n", motorReq[0], motorReq[1], motorReq[2], motorReq[3], motorReq[4], motorReq[5], motorReq[6], motorReq[7], motorReq[8], motorReq[9]);
			writeDebugStream("s%d;K\n",sensor1);
		}

		wait1Msec(HASH_SPEED);
	}

}


task pidController()
{
	float  pidSensorCurrentValue;

	float  pidError;
	float  pidLastError;
	float  pidIntegral;
	float  pidDerivative;
	float  pidDrive; // power level sent to launcher motors
	// Init the variables - thanks
	pidLastError  = 0; // last error/differnce of requested rpm value vs actual real value
	pidIntegral   = 0; //
	while( true )
	{
		//if (RPM > 3500 / PID_SENSOR_SCALE){
		//	intake(127);
		//}
		if( pidRPM ) // If PID control active-
		{ // -than do this
			// Read the sensor value and scale
			pidSensorCurrentValue = RPM;
			// calculate error/differnce
			pidError = pidSensorCurrentValue - pidRPMREQ;
			// integral - if Ki is not 0
			if( pid_Ki != 0 ) // if Ki number value is not equal to zero-
			{ // -than do this
				// If we are inside controlable window then integrate the error
				if( abs(pidError) < (PID_INTEGRAL_LIMIT) ) // if the absolute value of the piderror is less than the PID_INTEGRAL_LIMIT than
					pidIntegral = pidIntegral + pidError; // pidIntegral is equal to itself plus the pidError
				else
				{
					pidIntegral = 0; // pidIntegral is 0
				}
			}
			else
			{
				pidIntegral = 0; // pidIntegral is 0
			}
			// calculate the derivative
			pidDerivative = pidError - pidLastError;
			pidLastError  = pidError;
			// calculate drive
			pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);
			// limit drive
			if( pidDrive > PID_DRIVE_MAX ){
				pidDrive = PID_DRIVE_MAX;
			}
			//else
			//SensorValue[led] = 0;
			if( pidDrive < -127 )
				pidDrive = -127; // pid
			// send to motor
			intake(pidDrive * PID_MOTOR_SCALE); // runs the launcher at full power
		}
		else
		{
			// clear all
			pidError      = 0;
			pidLastError  = 0;
			pidIntegral   = 0;
			pidDerivative = 0;
			intake(0);
		}
		// Run at 50Hz
		wait1Msec( 25 );
	}
}
