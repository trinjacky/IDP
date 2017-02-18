//Software for Team L108, IDP (total lines of code: 580)

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
const int right_deviation[2] = {0x01, 0x03}; //LF LED: ON(BLACK)=0; OFF(WHITE)=1
const int left_deviation[2] = {0x04, 0x06};  //Sensor LSB(1): left; MSB(3): right;
const int reach_white_line = 0x07;			 
const double robot_length = 265;	 		 //select to prevent wall crash
const double robot_width = 270;
const double full_speed=40*80*pi/60000; 	 //in mm/ms
const int OVER_DRIVEN = 1; 					 //Error codes
const int OVER_TURNED = 2;
const int ALL_DARK = 3;
const int fruit_location[2][4] = {{70,50,75,75},{70,50,75,75}};

//Things needed to be calibrated
//Positioning, motor 3 time, color sensor threshold and position
//Colour sensor: Green LED TOP!!!
const double rev_dist[2] = {70,70};
const double pos_factor[2] = {1.2,1.2};
const int threshold[3] = {100,100,145};//{100,110,150};

int speed_compensation = 7; 			//compensate friction difference
int adjust_speed_addition = 10;				 
int previous_position;

int error_handling(int error_code, int motor_1_r, int motor_2_r);
int current_position();
double actual_speed(int rpm);
double calculate_time(int motor_r, double dist);
void robot_reverse(int motor_r, double distance);
void move_before_turn(int time_2, int motor_1_r, int motor_2_r);
int turn(char m, double close_time);
void robot_stop();

void check();
void go_to_first_stage(int motor_1_r, int motor_2_r, int run);
int drive(double time, int motor_1_r, int motor_2_r, int count_line, 
			  int trigger_handling);
int line_follow(int current_pos, int &count, int motor_1_r, int motor_2_r, 
					int error_case_trigger);
void fruit_picking(int i, int loop);
bool scan(int run, int fruit);
void claw(double time, int motor_3_r, char c);
void pick_up(int run, int vine, int fruit);
int dark_line(int motor_1_r, int motor_2_r, int count_line);
void go_to_second_stage(int motor_1_r, int motor_2_r, int tray);
int delivery();


int current_position()
{
	return rlink.request(READ_PORT_0) & 0x07;
}

double actual_speed(int rpm)
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

int delivery()
{
	rlink.command(WRITE_PORT_0,0x10 bitor current_position()); //tray DOWN
	delay(2000);
	rlink.command(WRITE_PORT_0,0x30 bitor current_position()); //tray UP
	delay(2000);
	return 0;
}

void check()
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
		cout << "Test failed (bad value returned)" << endl;                         
	rlink.command(WRITE_PORT_0,0x30 bitor current_position()); //arm UP
}

int turn(char m, double close_time)
{
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
					watch.stop();
					return 0;
				}
			}
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
					watch.stop();
					return 0;
				}
			}
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
		rlink.command(MOTOR_2_GO, motor_2_r+3.5*(count%3)+speed_compensation);
		count++;
	}
	else if(current_pos == right_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_compensation+adjust_speed_addition);
		count++;
	}
	else if(current_pos == right_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r-adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_compensation+adjust_speed_addition/2);
		count++;
	}
	else if(current_pos == left_deviation[0])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_compensation-adjust_speed_addition);
		count++;
	}
	else if(current_pos == left_deviation[1])
	{
		rlink.command(MOTOR_1_GO, motor_1_r+adjust_speed_addition/2);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_compensation-adjust_speed_addition/2);
		count++;
	}
	else if (current_pos == 0 && error_case_trigger == 1)
		return error_handling(ALL_DARK, 40, 40);
	else if (current_pos == 0)
	{
		rlink.command(MOTOR_1_GO, motor_1_r);
		rlink.command(MOTOR_2_GO,motor_2_r+3.5*(count%3)
							+speed_compensation);
		count++;
	}
	return 0;
}

void robot_reverse(int motor_r, double distance)
{
	adjust_speed_addition = -3;
	int time = calculate_time(motor_r%127, distance);
	watch.start();
	int count = 0;
	while(watch.read()<time)
		line_follow(current_position(), count, motor_r, motor_r,0);
	rlink.command(MOTOR_1_GO, 8);
	rlink.command(MOTOR_2_GO, 8);
	adjust_speed_addition = 10;
	watch.stop();
	delay(1000);
}

bool scan(int run, int fruit)
{
	bool val = false;
	int current_color = 0;
	int cutoff_time;
	if (fruit==1 && run==1)
		cutoff_time = 2000;
	else if (fruit==0 && run==0)
		cutoff_time = 1200;
	else
		cutoff_time = 1800;
	if (run==0)								//arm DOWN + sensor ON
		rlink.command(WRITE_PORT_0,0xE0 bitor current_position()); 
	else if (run==1)
		rlink.command(WRITE_PORT_0,0x60 bitor current_position());
	else
		rlink.command(WRITE_PORT_0,0xA0 bitor current_position());
	watch.start();
	while (!val && watch.read()<4000)   //what about lowest tomato?
	{
		current_color = rlink.request(ADC0);
		if (watch.read() > cutoff_time && current_color >= threshold[run])
		{
			cout<<"DETECTED"<<endl;
			val = true;
		}
	}
	rlink.command(WRITE_PORT_0,0x30 bitor current_position()); //arm UP + sensor OFF
	watch.stop();
	delay(2000);
	return val;
}

void fruit_picking(int i, int loop)
{
	int motor_r = 100;
	int count = 0;
	bool IS_TOMATO, IS_GREEN, IS_RED(false);
	for (int j=0;j<5;j++)
	{
		claw(500,120,'O');
		int current_pos;
		cout<<"Picking the "<<j<<"th fruit"<<endl;

		IS_TOMATO = scan(0,j);
		if (IS_TOMATO)
		{
			IS_RED = false;
			IS_GREEN = scan(1,j);
			if (IS_GREEN && loop == 1)
				IS_RED = scan(2,j);
			if (!IS_GREEN || IS_RED)	//result is red	
			{
				rlink.command(WRITE_PORT_1,~(0x0c^(4*(loop-1)))); //Indicating LED	
				pick_up(loop,i,j);
			}
			else  						//result is greem	
			{
				rlink.command(WRITE_PORT_1,~(0x14^(4*(loop-1)))); //Indicating LED
				delay(200);
			}
		}
		else
		{
			rlink.command(WRITE_PORT_1,~(0x44^(4*(loop-1))));	  //Indicating LED
			delay(200);
		}
		
		
		if (j<4)								//Moving on to next fruit
		{
			double time = calculate_time(motor_r, 
										 fruit_location[i][j]*pos_factor[i]);
			watch.start();
			while(watch.read()<time)
			{
				if(watch.read()<500)
					current_pos=at_the_middle;
				else
					current_pos=current_position();
				time+=line_follow(current_pos,count,motor_r,motor_r);
			}
			robot_stop();
			watch.stop();
		}		
		rlink.command(WRITE_PORT_1,~(0x04^(4*(loop-1)))); //Indicating LED
	}
}

void claw(double time, int motor_3_r, char c)
{
	switch(c)
	{
		case 'O':
			rlink.command(MOTOR_3_GO, motor_3_r+127); //claw OPEN
			delay(time);						   	  //should be fully open
			rlink.command(MOTOR_3_GO, 0);			  //stops opening
			break;
		case 'C':
			rlink.command(MOTOR_3_GO, motor_3_r); 	  //claw CLOSE
			delay(time);				  			  //closing time (empirical?)
			rlink.command(MOTOR_3_GO, 0);			  //stops closing
			break;
	}
}

void pick_up(int run, int vine, int fruit)
{
	rlink.command(RAMP_TIME, 0);
	int motor_3_r = 127;
	rlink.command(MOTOR_3_GO, 0);
	double time_s = 1500+run*500;		  	  
	delay(500);
	if (run==1 && vine==0 && fruit==0)
	{
		claw(time_s,motor_3_r,'C');
		claw(time_s,motor_3_r,'O');
	}
	claw(time_s,motor_3_r,'C');
	rlink.command(WRITE_PORT_0,0x20 bitor current_position()); //arm DOWN
	delay(2000);
	claw(time_s,motor_3_r,'O');
	rlink.command(WRITE_PORT_0,0x30 bitor current_position()); //arm UP
	delay(2000);	
}

int drive(double time, int motor_1_r, int motor_2_r, int count_line, 
				int trigger_handling=1)
{
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
		if ((watch.read()>2000&&watch.read()<7000)||(line_passed==1))//for ramp
			time += line_follow(current_position(),count,motor_1_r,
									motor_2_r,trigger_handling);
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
			watch.stop();
			return 1;
		}
	}
	counter.stop();
	return error_handling(OVER_DRIVEN, 187, 187);
}

int dark_line(int motor_1_r, int motor_2_r, int count_line)
{
	watch.start();
	int count = 0;
	int line_passed = 0;
	double time = calculate_time(motor_1_r, 2500.0);
	double time_2 = calculate_time(motor_1_r, robot_length); 
	while(watch.read()<time)
	{
		if (line_passed==1)   		//entered dark area
		{
			speed_compensation=5;	//not sure why but worked out best
			line_follow(at_the_middle,count,motor_1_r,motor_2_r,0);
		}			
		else  						//white line area of blind line
			line_follow(current_position(),count,motor_1_r,motor_2_r);
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
			speed_compensation = 10;
			cout << "READY TO TURN" << endl;
			count = 0;
			while (watch.read() < time_2)
				line_follow(at_the_middle, count, motor_1_r, motor_2_r, 0);
			watch.stop();
			return 1;
		}
	}
	return error_handling(OVER_DRIVEN,187,187);
}

void move_before_turn(int time_2, int motor_1_r, int motor_2_r)
{
	int count = 0;
	watch.start();
	while (watch.read() < time_2)
		line_follow(current_position(), count, motor_1_r, motor_2_r);
	watch.stop();
}

void go_to_first_stage(int motor_1_r, int motor_2_r, int run)
{
	rlink.command(RAMP_TIME,0);
	cout<<"Going to the picking stage"<<endl;
	double time_1 = calculate_time(motor_1_r, 1500.0);
	double time_2 = calculate_time(motor_1_r, robot_length);
	int next_move = drive(time_1, motor_1_r, motor_2_r, 3-run);
	for (int vine=0;vine<=1;vine++)
		if (next_move==1)
		{
			robot_reverse(127+50, rev_dist[vine]);
			fruit_picking(vine,run);
			next_move = drive(time_1, motor_1_r, motor_2_r, 1);
		}
	if (next_move==1)
	{
		move_before_turn(time_2, 100, 100);
		turn('L',1000);	
	}
}

void go_to_second_stage(int motor_1_r, int motor_2_r, int tray)
{
	cout<<"Going to the delivery stage"<<endl;
	double time_1 = calculate_time(motor_1_r, 2000.0);
	double time_2 = calculate_time(motor_1_r, robot_length);
	int next_move = drive(time_1, motor_1_r, motor_2_r, 3-tray);
	if (next_move==1)
	{
		move_before_turn(time_2/2.5, 100, 100);
		robot_stop();
		delivery();
	}
	next_move = drive(time_1, motor_1_r, motor_2_r, tray);
	if (next_move==1)
	{
		move_before_turn(time_2, 100, 100);
		turn('L',1000);	
	}
}

int error_handling(int error_code, int motor_1_r, int motor_2_r)
{
	cout<<"Entered error handling!"<<endl;
	int count = 0;
	double time = 500/actual_speed(motor_1_r%127);
	speed_compensation=0;
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
			}
			adjust_speed_addition=10;
			if (current_position()==reach_white_line)
			{
				speed_compensation=7;
				watch.stop(); 
				return 1;
			}
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
			if (current_position()==at_the_middle) 
			{
				watch.stop();
				return 1;
			}
			break;
		}
		case 3:
		{
			cout<<"Entered case 3: all-dark"<<endl;
			robot_stop();
			int start = watch.read();
			double turning_time = (pi/4*robot_width)/actual_speed(40);
			while(current_position()!=at_the_middle && 
					  watch.read()-start<turning_time)
			{
				if (previous_position==right_deviation[0]||
						previous_position==right_deviation[1])
					rlink.command(MOTOR_1_GO,127+motor_1_r);
				else
					rlink.command(MOTOR_2_GO,127+motor_2_r);
			}
			rlink.command(BOTH_MOTORS_GO_SAME,0);
			while(current_position()!=at_the_middle && 
					  watch.read()-start<3*turning_time)
			{
				if (previous_position==right_deviation[0]||
						previous_position==right_deviation[1])
					rlink.command(MOTOR_2_GO,127+motor_1_r);
				else
					rlink.command(MOTOR_1_GO,127+motor_2_r);
			}
			return watch.read()-start;
		}
	}
	return 1;
}

int main()
{	
	check();
	int motor_1_r=100;
	int motor_2_r=100;
	int next_move;
	for (int loop=1;loop<=2;loop++)
	{
		if (loop==1)
			rlink.command(WRITE_PORT_1,~0x04);	//Indicating LED
		else
			rlink.command(WRITE_PORT_1,~0x00);	//Indicating LED
		watch.start();
		go_to_first_stage(motor_1_r, motor_2_r,loop);
		next_move = dark_line(motor_1_r, motor_2_r, 2);
		if (next_move==1)
			turn('L',1000);
		go_to_second_stage(motor_1_r, motor_2_r, loop);
		next_move = drive(calculate_time(motor_1_r, 2300), motor_1_r, 
							  motor_2_r, 7-2*loop, 0); 
		move_before_turn(calculate_time(motor_1_r, robot_length),
							 motor_1_r, motor_2_r);
		if (loop==1 && next_move==1)
			turn('L',3000);
		watch.stop();
	}
	robot_stop();
	return 0;
}
