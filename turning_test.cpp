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
const double robot_length = 265;	 		 //select to prevent wall crash
const double robot_width = 275;
const double full_speed=40*80*pi/60000; 	 //in mm/ms
const int OVER_DRIVEN = 1; 					 //Error codes
const int OVER_TURNED = 2;
const int ALL_DARK = 3;
const int fruit_location[2][4] = {{75,60,85,85},{90,80,85,85}};
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

double calculate_time(int motor_r, double dist)
{
	return dist/(actual_speed(motor_r)*1.0);
}

void robot_stop()
{
	rlink.command(MOTOR_1_GO, 135);
	rlink.command(MOTOR_2_GO, 135);
}

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
	rlink.command(WRITE_PORT_0,0x30 bitor current_position());
}

int turn (char m, double close_time)
{
	//The idea is to set maximum angle (127 for 1 front wheel) and check
	//sensor in the middle or not. Whichever comes first stops the 
	//turning. 
	//Need a larger rotating speed to overcome inertia of front wheel(s)
	//which is why simply set the angle to 90 degrees won't work. Thus
	//should allow a longer time frame by setting turning_time slightly 
	//larger than it should.
	int turning_rpm = 100;
	double angle_rad = 140 * (pi/180);
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
				if (current_position()==at_the_middle && watch.read()>=close_time)
				{
					robot_stop();
					return 0;
				}
			}
			watch.stop();
			return error_handling(OVER_TURNED, turning_rpm, turning_rpm+127);
			break;
		}
		case 'R':
		{
			watch.start();
			while(watch.read()<turning_time)
			{
				rlink.command(MOTOR_1_GO,turning_rpm);
				rlink.command(MOTOR_2_GO,127+turning_rpm);
				if (current_position()==at_the_middle && watch.read()>=close_time)
				{
					robot_stop();
					return 0;
				}
			}
			watch.stop();
			return error_handling(OVER_TURNED, turning_rpm+127, turning_rpm);
			break;
		}
	}
	cout<<"No specified direction!"<<endl;
	return 0;
}

int line_follow(int current_pos, int &count, int motor_1_r, int motor_2_r, 
				int error_case_trigger=1)
{
	if (current_pos == at_the_middle)
	{	
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO, motor_2_r+3.5*(count%3)+speed_conpensation);
		count++;
	}
	else if(current_pos == right_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_conpensation+adjust_speed_addition);
		count++;
	}
	else if(current_pos == right_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_conpensation+adjust_speed_addition/2);
		count++;
	}
	else if(current_pos == left_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_conpensation-adjust_speed_addition);
		count++;
	}
	else if(current_pos == left_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_conpensation-adjust_speed_addition/2);
		count++;
	}
	else if (current_pos == 0 && error_case_trigger == 1)
	{
		cout<<"ALL DARK"<<endl;
		return error_handling(ALL_DARK, 40, 40);
	}
	else if (current_pos == 0)
	{
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_conpensation);
		count++;
	}
	return 0;
}

int drive_1(double time, int motor_1_r, int motor_2_r, int count_line, 
				int trigger_handling=1)
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
		if(counter.read()>100)
		{
			previous_position = current_position();
			counter.stop();
			counter.start();
		}
		if ((watch.read()>2000&&watch.read()<7000)||(line_passed==1))
			time += line_follow(current_position(),count,motor_1_r,motor_2_r,trigger_handling);
		else
			time += line_follow(current_position(),count,motor_1_r,motor_2_r);
		if(current_position() == reach_white_line && line_passed < count_line)
		{
			line_passed ++;
			cout << line_passed << endl;
			if (line_passed <= count_line-1)
				delay(1500);
		}
		if(line_passed >= count_line)
		{
			cout << "Passed" << endl;
			watch.stop();
			return 1;
		}
	}
	counter.stop();
	return error_handling(OVER_DRIVEN, 187, 187);
	//return 0;
}

void move_before_turn(int time_2, int motor_1_r, int motor_2_r)
{
	int count = 0;
	watch.start();
	while (watch.read() < time_2)
		line_follow(current_position(), count, motor_1_r, motor_2_r);
}

int error_handling(int error_code, int motor_1_r, int motor_2_r)
{
	cout<<"Entered error handling!"<<endl;
	int count = 0;
	double time = 500/actual_speed(motor_1_r%127);
	speed_conpensation=0;
	switch (error_code)
	{
		case 1:
		{
			adjust_speed_addition = -10;
			cout<<"Entered case 1: over-driven"<<endl;
			int error_case_trigger = 0;
			watch.start();
			while(current_position()!=reach_white_line && watch.read()<=time)
			{
				line_follow(current_position(), count, motor_1_r, motor_2_r,
				 error_case_trigger);
//				cout<<rlink.request(MOTOR_1)<<"	"<<rlink.request(MOTOR_2)<<endl;
			}
			adjust_speed_addition=10;
			if (current_position()==reach_white_line) 
			{
				return 1;
			}
			/*else 
			{
				adjust_speed_addition*=-1;
				error_handling(1, (motor_1_r+127)%254, (motor_2_r+127)%254);
			}*/
			break;
		}
		case 2:
		{
			cout<<"Entered case 2: over-turned"<<endl;
			watch.start();
			while(current_position()!=at_the_middle && watch.read()<=time/2)
			{
				rlink.command(MOTOR_2_GO,motor_2_r);
				rlink.command(MOTOR_1_GO,motor_1_r);
			}
			if (current_position()==at_the_middle) return 1;
			break;
		}
		case 3:
		{
			cout<<"Entered case 3: all-dark"<<endl;
			robot_stop();
			int start = watch.read();
			double turning_time = (pi/4*robot_width)/actual_speed(40);
			while(current_position()!=at_the_middle && watch.read()-start<turning_time)
			{
				if (previous_position==right_deviation[0]||previous_position==right_deviation[1])
				{
					rlink.command(MOTOR_1_GO,127+motor_1_r);
				}
				else
				{
					rlink.command(MOTOR_2_GO,127+motor_2_r);
				}
			}
			while(current_position()!=at_the_middle && watch.read()-start<3*turning_time)
			{
				if (previous_position==right_deviation[0]||previous_position==right_deviation[1])
				{
					rlink.command(MOTOR_2_GO,127+motor_1_r);
				}
				else
				{
					rlink.command(MOTOR_1_GO,127+motor_2_r);
				}
			}
			return watch.read()-start;
			break;
		}
	}
	return 0;
}

int main()
{	
	int motor_1_r=100;
	int motor_2_r=100;
	check();
	for (int i=1;i<=2;i++)
	{
		watch.start();
		int next_move = drive_1(calculate_time(motor_1_r, 2300), motor_1_r, motor_2_r, 5-i, 0);
		move_before_turn(calculate_time(motor_1_r, robot_length),motor_1_r, motor_2_r);
		if (i==1 && next_move==1)
			turn('L',2000);
	}		
	return 0;
}
