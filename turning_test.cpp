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
const int at_the_middle=0x05;
const int left_deviations[2] = {0x04, 0x06}; 
const int right_deviation[2] = {0x03, 0x01};
const int reach_white_line = 0x00;
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
	int val = rlink.request (TEST_INSTRUCTION);   // send test instruction
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
	double turning_time = (pi*robot_width/4)/actual_speed(turning_rpm);
	switch (m)
	{
		case 'L':
		{
			watch.start();
			while(watch.read()<turning_time)
			{
				rlink.command(MOTOR_2_GO,turning_rpm);
				rlink.command(MOTOR_1_GO,127+turning_rpm);
			}
			watch.stop();
			break;
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
			break;
		}
	}
}

int main()
{
	check();
	int count = 0;
	watch.start();
	while(watch.read()<3000)
	{
		rlink.command(MOTOR_1_GO, 90);
		rlink.command(MOTOR_2_GO, 90+4*(count%5)+10);
		count++;
	}
	watch.stop();
	turn('L');
	watch.start();
	while(watch.read()<3000)
	{
		rlink.command(MOTOR_1_GO, 90);
		rlink.command(MOTOR_2_GO, 90+4*(count%5)+10);
		count++;
	}
	turn('R');
	return 0;
}

	
