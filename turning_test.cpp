#include <iostream>
#include <robot_instr.h>
#include <stopwatch.h>
#include <robot_link.h>
#include <cmath>
using namespace std;
#define ROBOT_NUM  50                         // The id number (see below)
robot_link  rlink;                            // datatype for the robot link
stopwatch watch;
const double pi=3.1415926535897932;
//const int bit0 = 0x01;
//const int bit1 = 0x02;
//const int bit2 = 0x04;
const double robot_length = 285;
const double robot_width = 275;
const double full_speed=40*80*pi/60000; //in mm/ms

void pause (int period)
{
	watch.start();
	while(watch.read()<period)
	{
		continue;
	}
}

void check ()
{
	if (!rlink.initialise (ROBOT_NUM)) // setup the link
	{      
		cout << "Cannot initialise link" << endl;
		rlink.print_errs("    ");
	}
	val = rlink.request (TEST_INSTRUCTION);   // send test instruction
	if (val == TEST_INSTRUCTION_RESULT)	      // check result
		cout << "Test passed" << endl;
	else if (val == REQUEST_ERROR) 
	{
		cout << "Fatal errors on link:" << endl;
		rlink.print_errs();
	}
	else   									  // error, finish
	{
		cout << "Test failed (bad value returned)" << endl;
	}                             
}
	
double actual_speed (int rpm)
{
	if (rpm<=127)
		return rpm/127.0*full_speed;
	else
		return (127-rpm)/127.0*full_speed;
}

void turn (char m)
{
	int turning_rpm = 60;
	double angle_rad = pi * (100/360);
	double turning_time = (angle_rad*robot_width/2)/actual_speed(turning_rpm);
	switch (m)
	{
		case 'L':
		{
			watch.start();
			while(watch.read()<turning_time)
			{
				rlink.command(MOTOR_1_GO,turning_rpm);
				rlink.command(MOTOR_2_GO,127+turning_rpm);
			}
			watch.stop();
		}
		case 'R':
		{
			watch.start();
			while(watch.read()<turning_time)
			{
				rlink.command(MOTOR_1_GO,turning_rpm);
				rlink.command(MOTOR_2_GO,127+turning_rpm);
			}
			watch.stop();
		}
	}
}

int main()
{
	check();
	int count = 0;
	while(watch.read()<3000)
	{
		rlink.command(MOTOR_1_GO, 90);
		rlink.command(MOTOR_2_GO, 90+4*(count%5)+10);
	}
	turn('L');
	watch.start();
	while(watch.read()<3000)
	{
		rlink.command(MOTOR_1_GO, 90);
		rlink.command(MOTOR_2_GO, 90+4*(count%5)+10);
	}
	watch.stop();
	turn('R');
	return 0;
}

	
