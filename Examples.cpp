#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <stopwatch.h>
#include <robot_link.h>
#include <cmath>
#define ROBOT_NUM  50                     // The id number (see below)
robot_link  rlink;                        // datatype for the robot link
stopwatch watch;
//Line-following LED: ON(BLACK)=0; OFF(WHITE)=1
const double pi=3.1415926535897932384626;
const int at_the_middle = 0x02;
const int left_deviation[2] = {0x01, 0x03}; 
const int right_deviation[2] = {0x04, 0x06};
const int reach_white_line = 0x07;
//const int special_case = 0x02;
int speed_conpensation = 10;
int adjust_speed_addition = 10;
const double robot_length = 285;
const double robot_width = 275;
const double full_speed=40*80*pi/60000; //in mm/ms

void pause(int period)
{
	watch.start();
	while(watch.read()<period)
	{
		continue;
	}
}

int current_position()
{
	return rlink.request(READ_PORT_0) & 0x07;
}

int exception_handling(int Exception_number);
int fruit_picking();
int ramp_climbing();
int delivery();
int returning();

void check ()
{
	if (!rlink.initialise (ROBOT_NUM))			// setup the link
	{      
		cout << "Cannot initialise link" << endl;
		rlink.print_errs("    ");
	}
	int val = rlink.request (TEST_INSTRUCTION); // send test instruction
	if (val == TEST_INSTRUCTION_RESULT)	        // check result
		cout << "Test passed" << endl;
	else if (val == REQUEST_ERROR) 
	{
		cout << "Fatal errors on link:" << endl;
		rlink.print_errs();
	}
	else   									    // error, finish
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
	//The idea is to set maximum angle (127 for 1 front wheel) and check
	//sensor in the middle or not. Whichever comes first stops the 
	//turning. 
	//Need a larger rotating speed to overcome inertia of front wheel(s)
	//which is why simply set the angle to 90 degrees won't work. Thus
	//should allow a longer time frame by setting turning_time slightly 
	//larger than it should.
	
	int turning_rpm = 100;
	double angle_rad = 127 * (pi/180);
	double turning_time = (angle_rad*robot_width/2)/actual_speed(turning_rpm);
	cout<<turning_time<<endl;
	switch (m)
	{
		case 'L':
		{
			watch.start();
			while(watch.read()<turning_time)
			{
				rlink.command(MOTOR_2_GO,turning_rpm);
				rlink.command(MOTOR_1_GO,127+turning_rpm);
				if (current_position()==at_the_middle && watch.read()>=1000)
					break;
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
				if (current_position()==at_the_middle && watch.read()>=1000)
					break;
			}
			watch.stop();
			break;
		}
	}
}

void drive_1(double time, double time_2, int motor_1_r, int motor_2_r, char turn_direction)
{
	//Read the current position. Decide whether to go straight, turn 
	//slightly left or right, or raise an error because all sensors
	//detect black line. If all sensors detect write line, call the turn
	//function TODO Consider the use of the sensor at the tail
	watch.start();
	int current_pos = current_position();
	int count=0;
	while(watch.read()<time)
	{
		current_pos = current_position();
		cout<<current_pos<<endl;
		if (current_pos == at_the_middle)
		{	
			rlink.command(MOTOR_1_GO, motor_1_r);
			rlink.command(MOTOR_2_GO, motor_2_r+4*(count%5)+speed_conpensation);
			count++;
		}
		else if(current_pos == left_deviation[0] || current_pos == left_deviation[1])
		{
				rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition);
				rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)
							+speed_conpensation+adjust_speed_addition);
				count++;
		}
		else if(current_pos == right_deviation[0] || current_pos == right_deviation[1])
		{
				rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition);
				rlink.command(MOTOR_2_GO,motor_2_r+4*(count%5)
							+speed_conpensation-adjust_speed_addition);
				count++;
		}
		else if(current_pos == reach_white_line)
		{
			watch.stop();
			watch.start();
			count = 0;
			while (watch.read() < time_2)
			{
				rlink.command(MOTOR_1_GO, motor_1_r);
				rlink.command(MOTOR_2_GO, motor_2_r+4*(count%5)+speed_conpensation);
				count++;
			}
			turn(turn_direction);
			break;
		}
		else
		{
			watch.stop();
			cout<<"error occur!";
			continue;
		}
	}
}

void go_to_first_stage(int &count)
{
	int motor_1_r=90;
	int motor_2_r=90;
	double motor_1_v=actual_speed(motor_1_r); 
	double motor_2_v=actual_speed(motor_2_r);
	double distance = 5000.0;
	double time_1=distance/motor_1_v;
	double time_2 = robot_length/motor_1_v;
	drive_1(time_1, time_2, motor_1_r, motor_2_r, 'R');
}

int main ()
{
	check();
	int count=0;
	go_to_first_stage(count);
	return 0;
}
