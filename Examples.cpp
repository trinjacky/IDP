#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <stopwatch.h>
#include <robot_link.h>
#include <delay.h>
#define ROBOT_NUM  50                     	 // The id number on WiFi card
robot_link rlink;                        	 // datatype for the robot link
stopwatch watch;
stopwatch counter;
const double pi=3.1415926;
const int at_the_middle = 0x02;
const int right_deviation[2] = {0x01, 0x03}; //Line-following LED: ON(BLACK)=0; OFF(WHITE)=1
const int left_deviation[2] = {0x04, 0x06};  //Sensor LSB(1): left; MSB(3): right
const int reach_white_line = 0x07;
//const int special_case = 0x02;
const double robot_length = 250;	 		 //select to prevent wall crash
const double robot_width = 275;
const double full_speed=40*80*pi/60000; 	 //in mm/ms
const int TURN = 1;
const int GO_AHEAD = 0;
const int OVER_DRIVEN = 1;
const int OVER_TURNED = 2;
const int THREE_BLACK = 3;
int speed_conpensation = 10; 				 //compensate friction difference
int adjust_speed_addition = 10;				 //
int previous_position;

int current_position()
{
	return rlink.request(READ_PORT_0) & 0x07;
}

double actual_speed (int rpm)
{
	if (rpm<=127)
		return rpm/127.0*full_speed;
	else
		return (rpm-127)/127.0*full_speed;
}

int exception_handling(int Exception_number);

int fruit_picking()
{
	return 0;
}

int delivery();
int returning();
int error_handling(int error_code, int motor_1_r, int motor_2_r);


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

int turn (char m)
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
					return 0;
			}
			watch.stop();
			return error_handling(2, turning_rpm, turning_rpm+127);
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
					return 0;
			}
			watch.stop();
			return error_handling(2, turning_rpm+127, turning_rpm);
			break;
		}
	}
	cout<<"No specified direction!"<<endl;
	return 0;
}

void line_follow(int current_pos, int &count, int motor_1_r, int motor_2_r) //parameters may only be suitable for 90rpm following
{
	if (current_pos == at_the_middle)
	{	
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO, motor_2_r+3.5*(count%5)+speed_conpensation);
		count++;
	}
	else if(current_pos == right_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%5)
							+speed_conpensation+adjust_speed_addition);
		count++;
	}
	else if(current_pos == right_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%5)
							+speed_conpensation+adjust_speed_addition/2);
		count++;
	}
	else if(current_pos == left_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%5)
							+speed_conpensation-adjust_speed_addition);
		count++;
	}
	else if(current_pos == left_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%5)
							+speed_conpensation-adjust_speed_addition/2);
		count++;
	}
	else
	{
		cout<<"ALL DARK"<<endl;
		error_handling(3, 40, 40);
	}
}

int drive_1(double time, double time_2, int motor_1_r, int motor_2_r, int count_line)
{
	//Read the current position. Decide whether to go straight, turn 
	//slightly left or right, or raise an error because all sensors
	//detect black line. If all sensors detect write line, call the turn
	//function TODO Consider the use of the sensor at the tail
	watch.start();
	int count = 0;
	int line_passed = 0;
	counter.start();
	while(watch.read()<time)
	{
		if(counter.read()>1000)
		{
			previous_position = current_position();
			counter.stop();
			counter.start();
		}
		line_follow(current_position(),count,motor_1_r,motor_2_r);
		if(current_position() == reach_white_line && line_passed < count_line)
		{
			line_passed ++;
			cout << line_passed << endl;
			if (line_passed <= count_line-1)
				delay(1000);
		}
		else if(line_passed >= count_line)
		{
			watch.stop();
			watch.start();
			cout << "READY TO TURN" << endl;
			return 1;
		}
	}
	counter.stop();
	int return_value = error_handling(1, 157, 157);
	return return_value;
}

void move_before_turn(int time_2, int motor_1_r, int motor_2_r)
{
	int count = 0;
	watch.start();
	while (watch.read() < time_2)
		line_follow(current_position(), count, motor_1_r, motor_2_r);
}

int dark_line (double time, double time_2, int motor_1_r, int motor_2_r, int count_line)
{
	watch.start();
	int count = 0;
	int line_passed = 0;
	while(watch.read()<time)
	{
		if (current_position()!=0 || watch.read()<5000) //white line area of blind line
			line_follow(current_position(),count,motor_1_r,motor_2_r);
		else    										//entered dark area
		{
			speed_conpensation=0;						//not sure why but worked out best
			line_follow(at_the_middle,count,motor_1_r,motor_2_r);
			speed_conpensation=10;
		}			
		if(current_position() == reach_white_line && line_passed < count_line)
		{
			line_passed ++;
			cout << line_passed << endl;
			if (line_passed <= count_line-1)
				delay(1000);
		}
		if(line_passed >= count_line)
		{
			watch.stop();
			watch.start();
			cout << "READY TO TURN" << endl;
			int count = 0;
			while (watch.read() < time_2)
				line_follow(at_the_middle, count, motor_1_r, motor_2_r);
			return 1;
		}
	}
	return -1;
}

void go_to_first_stage()
{
	int motor_1_r=90;
	int motor_2_r=90;
	double motor_1_v=actual_speed(motor_1_r); 
	double distance = 2300.0;
	double time_1=distance/motor_1_v;
	double time_2 = robot_length/motor_1_v;
	int next_move = 0;
	next_move = drive_1(time_1, time_2, motor_1_r, motor_2_r, 4);
	cout << next_move << endl;
	if (next_move==1)
	{
		move_before_turn(time_2, 90, 90);
		turn('L');
	}
	next_move = dark_line(time_1, time_2, motor_1_r, motor_2_r, 2);
	if (next_move==1)
	{
		turn('L');
	}
	next_move = drive_1(time_1, time_2, motor_1_r, motor_2_r, 3);
	if (next_move==1)
	{
		move_before_turn(time_2, 90, 90);
		turn('L');	
	}
	next_move = drive_1(time_1, time_2, motor_1_r, motor_2_r, 3);
	move_before_turn(time_2, 90, 90);
	
}

int error_handling(int error_code, int motor_1_r, int motor_2_r)
{
	cout<<"entered the error handling!"<<endl;
	int count = 0;
	double time = robot_length/actual_speed(motor_1_r);
	switch (error_code)
	{
		case 1:
		{
			watch.start();
			while(current_position()!=reach_white_line && watch.read()<=time)
			{
				line_follow(current_position(), count, motor_1_r, motor_2_r);
			}
			if (current_position()==reach_white_line) return 1;
			else error_handling(1, motor_1_r+127, motor_2_r+127);
			break;
		}
		case 2:
		{
			watch.start();
			while(current_position()!=at_the_middle && watch.read()<=time/2)
			{
				rlink.command(MOTOR_2_GO,127+motor_1_r);
				rlink.command(MOTOR_1_GO,motor_2_r);
			}
			if (current_position()==at_the_middle) return 1;
			else error_handling(1, motor_1_r+127, motor_2_r+127);
			break;
		}
		case 3:
		{
			rlink.command (BOTH_MOTORS_GO_SAME,0);
			while(current_position()!=at_the_middle)
			{
				if (previous_position==right_deviation[0]||previous_position==right_deviation[1])
					rlink.command(MOTOR_1_GO,127+motor_1_r);
				else
					rlink.command(MOTOR_2_GO,127+motor_2_rs);
			}
				return 1;
		}
			
	}
	return 0;
}

int main ()
{	
	check();
	go_to_first_stage();
	fruit_picking();
	return 0;
}
