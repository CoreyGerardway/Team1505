//This file contains all the basic movement functions that robot can use.
//Copy and modify this file based on the configuration of your robot.
//Basic names for motors are given below.

/************************************************************************************************

leftFrontWheel
leftRearWheel
leftLowerArm
leftUpperArm
leftIntake
rightFrontWheel
rightRearWheel
rightLowerArm
rightUpperArm
rightIntake


intakeBack
rightDrive
leftDrive
intakeFront
leftArm
rightFrontL
rightBackL
leftFrontL
leftBackL
rightArm



******************************************************************************************************/

//The basic drive command
//This is used for skid steering.
//The left and right are independent.

void drive(int leftM, int rightM){
	if (abs(leftM) < MOTOR_DEADBAND)
		leftM = 0;
	if (abs(rightM) < MOTOR_DEADBAND)
		rightM = 0;
	motorReq[testOne] = rightM;
	motorReq[testTwo] = leftM;
	motorReq[testThree] = rightM;
	motorReq[testFour] = leftM;
}//end of void drive

//This function uses our basic drive controls and allows you to use arcade style steering
void logBaseControl(float straight, float turn){
	if (abs(straight) < MOTOR_DEADBAND)
		straight = 0;
	if (abs(turn) < MOTOR_DEADBAND)
		turn = 0;

	//// Logarithmic control ////
	if(straight > 0){ straight = straight*straight/127;}
	else{ straight = -(straight*straight/127);}

	if(turn > 0){ turn = turn*turn/127;}
	else{ turn = -(turn*turn/127);}
	//Change the + and negative signs below if robot is turning the wrong way
	drive(straight + turn, straight - turn);
}

//Code created for the math of mecanum drive
//This takes into account all the options
void mecanum(int forward,int turn,int strafe){
	if (abs(forward) < MOTOR_DEADBAND)
		forward = 0;
	if (abs(turn) < MOTOR_DEADBAND)
		turn = 0;
	if (abs(strafe) < MOTOR_DEADBAND)
		strafe = 0;
	//	motorReq[rightFrontWheel] = forward - turn - strafe;
	//	motorReq[rightRearWheel] = forward + turn - strafe;
	//	motorReq[leftFrontWheel] =  forward + turn + strafe;
	//	motorReq[leftRearWheel] =  forward - turn + strafe;
}

//This is a basic two motor lifiting function
//remove or rename the function if your lift is different.
void armLift(int power){
	if (abs(power) < MOTOR_DEADBAND)
		power = 0;
//	motorReq[rightArm] = power;
//	motorReq[leftArm] = power;
}

//Two motor intake.
//Change or modify again if you have a different intake.
void intake(int power){
	if (abs(power) < MOTOR_DEADBAND)
		power = 0;
	if( power < PID_DRIVE_MIN )
  	power = PID_DRIVE_MIN;
	motorReq[testOne] = power;
	motorReq[testTwo] = power;
	motorReq[testThree] = power;
	motorReq[testFour] = power;
	//motorReq[spool] = power;
}

void belts(int power){ // IS THE REAL INTAKE
	if (abs(power) < MOTOR_DEADBAND)
		power = 0;
	if(true){
		//motorReq[intakeFront] = power;
		//motorReq[intakeBack] = power;
		//motorReq[intake2] = power;
	}
}
