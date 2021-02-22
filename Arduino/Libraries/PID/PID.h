// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID CONTROLLER HEADER FILE.

class PID
{
	public:

	// Motor commanded power.
	double right_speed;
	double left_speed;
	
	// Tick counters.
	int right_ticks;
	int left_ticks;
	double M1_ticks_moved;
	double M2_ticks_moved;
	double M1_ticks_diff;
	double M2_ticks_diff;
	double Total_M1_moved;
	double Total_M2_moved;
	double M1_ticks_to_move;    // Individual number of ticks to move is specified in each scenario.
	double M2_ticks_to_move;
	double M1_setpoint_ticks;   // Number of ticks before each iteration of PID controller, also controls motor speed.
	double M2_setpoint_ticks;   // A larger value here corresponds to the robot moving faster.

	// Proportional Integral Derivative (PID) controller.
	double M1_ticks_PID;
	double M2_ticks_PID;
	double E1_error_ticks;
	double E2_error_ticks;
	double E1_prev_error;
	double E2_prev_error;
	double E1_sum_error;
	double E2_sum_error;
	double KP;        	// Adjust for proportional component.
	double KD;        	// Adjust for derivative component.
	double KI;          // Adjust for integral component.
	
	// Initialization of PID controller.
	void init();
	
	// Tick incrementing functions.
	void right_ticks_increment(void);
	void left_ticks_increment(void);
	
	// PID controller.
	void control(int right_mul , int left_mul);
	
	// Conversion of ticks to motor power.
	double right_ticks_to_power(double right_ticks);
	double left_ticks_to_power(double left_ticks);
	
	// Stores the number of ticks.
	int getRightSpeed();
	int getLeftSpeed();
	
	// Reset the PID controller values.
	void setZero();
};