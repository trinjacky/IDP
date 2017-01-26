#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <stopwatch.h>
#include <robot_link.h>
#include <cmath>
#define ROBOT_NUM  50                         // The id number (see below)
robot_link  rlink;                            // datatype for the robot link
stopwatch watch;
const double pi=3.1415926535897932;
const int at_the_middle=0x05;
const int left_deviations[2] = {0x04, 0x06}; 
const int right_deviation[2] = {0x03, 0x01};
const int reach_white_line = 0x00;
//const int special_case = 0x02;
int speed_conpensation = 10;
int adjust_speed_addition = 10

void pause(int period)
{
	watch.start();
	while(watch.read()<period)
	{
		continue;
	}
}

void Exception_handling(int Exception_number)

void drive_1(int time, int motor_1_r, int motor_2_r, char turn_direction)
{
	watch.start();
	int current_pos = rlink.request(READ_PORT_0) & 0x07;
	while(watch.read()<time)
	{
		if (current_pos == at_the_middle)
		{	
			rlink.command(MOTOR_1_GO, motor_1_r);
			rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)+speed_conpensation);
			count++;
		}
		else if(current_pos == left_deviation[0] or current_pos == left_deviation[1])
		{
			while (current_pos != at_the_middle && watch.read()<time)
			{
				rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition);
				rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)
							+speed_conpensation+adjust_speed_addition);
				count++;
			}
		}
		else if(current_pos == right_deviation[0] or current_pos == right_deviation[1])
		{
			while (current_pos != at_the_middle && watch.read()<time)
			{
				rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition);
				rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)
							+speed_conpensation-adjust_speed_addition);
				count++;
			}
		}
		else if(current_pos == reach_white_line)
		{
			turn(turn_direction);
			break;
		}
		else
		{
			Exception_handling();
			break;
		}
	}
}

















int main ()
{
	watch.start();
	int   val;                                // data from microprocessor
	if (!rlink.initialise (ROBOT_NUM)) {      // setup the link
	cout << "Cannot initialise link" << endl;
	rlink.print_errs("    ");
	return -1;
	}
	val = rlink.request (TEST_INSTRUCTION);   // send test instruction
	if (val == TEST_INSTRUCTION_RESULT) {     // check result
	cout << "Test passed" << endl;
	cout<<watch.read()<<endl;
	int count=0;
	double full_velocity=40*80*pi/60000; 
	int motor_1_r=90;
	int motor_2_r=90;
	double motor_1_v=motor_1_r/127.0*full_velocity; 
	double motor_2_v=motor_2_r/127.0*full_velocity;
	cout<<motor_1_v<<endl;
	cout<<motor_2_v<<endl;
	double distance = 50.0;
	double time_1=distance/motor_1_v;
	while(watch.read()<time_1)
	{
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)+10);
		count++;
	}
	watch.start();
	while(watch.read()<6600)
	{
		rlink.command(MOTOR_1_GO,127+65);
		rlink.command(MOTOR_2_GO,60);
	}
	watch.start();
	while(watch.read()<5000)
	{
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)+10);
		count++;
	}
	cout<<rlink.request(MOTOR_1);                         // all OK, finish
	}
	else if (val == REQUEST_ERROR) {
	cout << "Fatal errors on link:" << endl;
	rlink.print_errs();
	}
	else
	cout << "Test failed (bad value returned)" << endl;
	return -1;                                // error, finish
}
