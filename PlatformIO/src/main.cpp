#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include <FastLED.h>
#include <math.h>

/*
We opted to not use the provided template code,
as we have additional goals for our robot.
As such, our code works very differently from the
provided code, but we have tried to document it as
best we could.

-

Our first goal was to avoid the calibration process
all together.

The first step was ensuring uniform and consistent
illumination of the track. This is accomplished
with a segment of an LED strip that is placed
next to the photoresistors.

The next step was to get the weighted average of the raw
photoresistor outputs and place the robot at 13 positions
from far left to far right.

We record what the raw photoresistor outputs were
and correspond them with the "actual" position
of the robot.

This gives us a lookup table that we can use to
calculate the position of the robot.

We went the extra step of approximating the
relationship with a regressed cubic polynomial.
(We found a cubic equation that followed the lookup
table close enough). This simplified the process.

-

Our second goal was to have absolutely no
potentiometers on the robot.

This, of course, means that we cannot change the PID
terms between tracks, so we have to make it adapt
to each track.

The first step is manually tuning the PID terms at
various speeds, slow to fast. We then ramp up the
speed of the robot as long as it is following the
track well. The PID terms in use are linearly
interpolated between the values we found previously.

So we have a LUT that looks like this:
Speed 1 -> PID 1
Speed 2 -> PID 2
...
Speed N -> PID N

For example, if the current speed is 1.5, then the
PID terms are half way between PID 1 and PID 2.

How do we know when to slow down?

We want to slow down when the track is quickly escaping
the center of our robot, indicating we are moving faster
than we can keep up. A greate measurement of this is
the change in error. So we decrease the speed
proportional to the current D term times dE/dt.
(At the moment we do not care about dt as it is basically
constant).

The final issue is abrupt turns. The robot often
runs off the track.

We haven't implemented this yet, but we plan
to counter this by making another LUT or
approximation of the relationship between the
sum of the raw photoresistor outputs and the
quantity of track visible.

If the robot is following a simple track, our
"confidence" value should be within the
appropriate range, if not, (meaning the robot
is either over a very steep turn, over multiple tracks,
or over no tracks at all) we continue motion
at the rate at the previous valid "confidence"
value.

It is difficult to explain this in text,
I hope I explained it well, but if there is any
confusion, please come to use and we can discuss
in person.

Thank you from the Roly Poly team!

*/

// CODE BEGIN

/*
TYPEDEFS
*/

// represent P, I, and D constants for the PID controller. R & S represent integration maximum and AFMS motor speed, respectively.
typedef struct
{
	float p;
	float i;
	float d;
	float r;
	float s;
} pid_t;

// maps observed position to real position
typedef struct
{
	float observed;
	float actual;
} map_t;

// representation of a position and "confidence" of the position
typedef struct
{
	float position;
	float spread;
} pseudoposition_t;

int sign(float x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

float clamp_abs(float x, float bound) {
	if (x > bound)
		return bound;
	else if (x < -bound)
		return -bound;
	else
		return x;
}

typedef enum
{
	ST_NOMINAL,
	ST_ALL_WHITE,
	ST_ALL_BLACK
} line_state_t;

/*
VARIABLE INITIALIZATIONS
*/

CRGB leds[5];
#define LEDS_PIN 6

// a LUT mapping current speed to predetermined corresponding PID constants
const pid_t speedLUT[] = {
	(pid_t){
		// Speed 0 (40)
		25,
		0.05,
		2,
		8,
		40},
	(pid_t){
		// Speed 1 (80)
		20,
		0.1,
		90,
		16,
		80},
	(pid_t){
		// Speed 2 (120)
		45,
		0.075,
		150,
		40,
		120},
	(pid_t){
		// Speed 3 (250)
		50,
		0.1,
		275,
		30,
		255},
	(pid_t){
		// Speed 3 (250)
		50,
		0.1,
		275,
		30,
		255}};

// unused
const map_t photomapLUT[] = {
	(map_t){}};

const int photo_pins_unmapped[] = {A8, A9, A10, A11, A12, A13, A14};
const int photo_pins[] = {A13, A11, A9, A12, A14, A8, A10};
// {0, 0, 1, 2, 3, 4, 5, 6, 7, 7}
const float photo_LUT[] = {0.58, 0.58, 0.74, 0.73, 0.35, -0.03, -0.11, 0.11, 0.11};

// motor shield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

float speed = 0.5;
int leftSpeed = 0, rightSpeed = 0;

float previous_pos; // previous position
float err_acc;		// accumulated error

line_state_t state;
line_state_t state_prev;

/*
FUNCTION DEFINTIONS
*/

// lerps PID LUT to provide appropriate PID terms for a given speed
pid_t pid_lerp(float n)
{
	int whole = (int)n;
	float frac = n - whole;
	float frac_inv = 1.0 - frac;

	pid_t sl = speedLUT[whole];
	pid_t sh = speedLUT[whole + 1];

	return (pid_t){
		sh.p * frac + sl.p * frac_inv,
		sh.i * frac + sl.i * frac_inv,
		sh.d * frac + sl.d * frac_inv,
		sh.r * frac + sl.r * frac_inv,
		sh.s * frac + sl.s * frac_inv};
}

int extreme_position_lock = 0;
int all_white_lock = 0;
int all_black_lock = 0;
const int lock_reset = 200;
bool bad_state = false;
bool bad_state_prev = false;

line_state_t find_state(int sum)
{
	if (sum < 2600)
		return ST_ALL_WHITE;
	if (sum < 3000)
		return ST_NOMINAL;
	else
		return ST_ALL_BLACK;
}

char *state_str(line_state_t state)
{
	switch (state)
	{
	case ST_NOMINAL:
		return "1 Line (Nominal)";
	case ST_ALL_BLACK:
		return "All Black";
	case ST_ALL_WHITE:
		return "All White";
	default:
		return "Bruh";
	}
}

// calculates a pseudoposition from photoresistor readings. This is raw and uncalibrated, thus unusable without real_position().
pseudoposition_t pseudoposition_calc()
{
	float position_raw = 0.0;
	int sum = 0;

	for (int i = 0; i < 7; i++)
	{
		float readout = 1024.0 - ((float)analogRead(photo_pins[i]));
		// Serial.print(readout);
		// Serial.print(" \t");
		float node = (float)(i - 3);
		float weight = node * readout;
		position_raw += weight;
		sum += readout;
	}
	//Serial.println();

	//Serial.println(position_raw);

	//Serial.println(sum);

	state = find_state(sum);
	if (state != state_prev)
	{
		//Serial.print("Change state: ");
		//Serial.println(state_str(state));
	}
	state_prev = state;

	/*
	if (sum > 2800 && sum < 3200)
		Serial.println("On Track");
	else
		Serial.println("Off Track");
		*/

	/*
  int whole = (int)position_raw + 3;
  float frac = position_raw + 3 - whole;
  float frac_inv = 1.0 - frac;

  float position_low = photo_LUT[whole + 1];
  float position_high = photo_LUT[whole + 2];

  float position = position_high * frac + position_low * frac;
  
  float summation = 0;
  for (int i = 0; i < 7; i++)
  {
	float diff = weights[i] - position_raw;
	summation += diff * diff;
  }

  float deviation = sqrtf(summation / 7);
  */
	return (pseudoposition_t){
		.position = position_raw,
		//.spread = deviation
		.spread = 0};
}

// converts a pseudoposition to a real position using a polynomial approximation of the mapping from pseudoposition to real position
// Eliminates the need for calibration!
float real_position(float pseudoposition)
{
	
	float x = pseudoposition;
	Serial.println(x);
	
	const float ys[] = {2,1.5,1,.5,0,-.5,-1,-1.5,-2};
	const float xs[] = {-100,-44,11,51,71,74,111,187,280};

	int xs_length = sizeof(xs) / sizeof(float);
	float diffs[xs_length-1] = {}; 
	for (int i = 0; i < xs_length - 1; i++)
	{
		diffs[i] = xs[i + 1] - xs[i];
	}
	int region = 0;
	while (x > xs[region+1])
	{
		region++;
		if (region == xs_length - 2)
			break;
	}
	float y_left = ys[region];
	float y_right = ys[region + 1];
	float diff = (x - xs[region])/diffs[region];
	float y = y_left + diff * (y_right - y_left);
	return y;
}

float leftSpeedValid, rightSpeedValid;
float curvature = 0.0;
int last_time;

line_state_t prev_state;

// get PID controller output given error and speed, using the PID constants from the LUT
void pid_step(float error, float *speed, int *leftSpeed, int *rightSpeed, float curvature)
{
	//Serial.println(error);
	
	float speed_limit = 3.0;
	float error_limit = 2.5;
	if (fabs(error) > error_limit)
	{
		*speed = 0;
		error = clamp_abs(error, error_limit);
	}

	float output = 0;
	pid_t terms = pid_lerp(*speed);
	output += terms.p * error;
	output += terms.i * err_acc;
	output += terms.d * (error - previous_pos);
	err_acc += error;
	err_acc = clamp_abs(err_acc, terms.r);
	float speed_change = 0.02 -abs(0.2 * terms.d * (error - previous_pos)) * 3 / 255;
	if (speed_change < 0.0) {
		output *= 3;
		speed_change *= 3;
	}
	*speed += speed_change;
	*speed = min(max(*speed, 0), speed_limit);


	if (output > 0)
	{
		*leftSpeed = terms.s;
		*rightSpeed = terms.s - (int)output;
		//.println(output);
		*rightSpeed = *rightSpeed < -255 ? -255 : *rightSpeed;
	}
	else
	{
		*leftSpeed = terms.s + (int)output;
		*rightSpeed = terms.s;
		*leftSpeed = *leftSpeed < -255 ? -255 : *leftSpeed;
	}

	if(state != prev_state) {
	
		if (state == ST_NOMINAL)
		{
			if (prev_state == ST_ALL_BLACK)
			{
				*leftSpeed = 255;
				*rightSpeed = 255;
			}
			leftSpeedValid = *leftSpeed;
			rightSpeedValid = *rightSpeed;
		}
		else if (state == ST_ALL_BLACK)
		{
			
		}
		else if (state == ST_ALL_WHITE)
		{
			*speed = 0;
		}
		prev_state = state;
	}

	//Serial.println(state_str(state));

	previous_pos = error;
}

// standard Arduino setup function
void setup()
{
	Serial.begin(9600);
	Serial.println("Starting...");
	FastLED.addLeds<WS2811, LEDS_PIN, GBR>(leds, 5);
	if (!AFMS.begin())
	{
		Serial.println("Motor shield not found");
		while (1)
			;
	}
	for (int i = 0; i < 5; i++)
	{
		leds[i] = CRGB::White;
		FastLED.show();
	}

	Motor2->setSpeed(0);
	Motor2->run(FORWARD);
	Motor4->setSpeed(0);
	Motor4->run(FORWARD);
}

// standard Arduino loop function
void loop()

{	
	int time = millis();

	int duration = time - last_time;
	last_time = time;

	pid_step(real_position(pseudoposition_calc().position), &speed, &leftSpeed, &rightSpeed, curvature);
	/*
	if (state == ST_ALL_BLACK) {
		leftSpeed = sign(curvature);
		rightSpeed = sign(curvature);
	}
	*/

	Motor2->setSpeed(abs(leftSpeed));
	Motor2->run(leftSpeed > 0 ? FORWARD : BACKWARD);
	Motor4->setSpeed(abs(rightSpeed));
	Motor4->run(rightSpeed > 0 ? FORWARD : BACKWARD);

	curvature = .9 * curvature + sign(leftSpeed - rightSpeed);

	
	// if (!extreme_position_lock && !all_white_lock && !all_black_lock)
	// 	bad_state = false;

	// if (all_white_lock)
	// {
	// 	all_white_lock--;
	// 	//if (!all_white_lock)
	// 	//	Serial.println("All White Unlocked.");
	// }

	// if (all_black_lock)
	// {
	// 	all_black_lock--;
	// 	//if (!all_black_lock)
	// 	//	Serial.println("All White Unlocked.");
	// }
	// if (extreme_position_lock)
	// {
	// 	extreme_position_lock--;
	// 	//if (!extreme_position_lock)
	// 	//	Serial.println("Extreme Position Unlocked.");
	// }

}