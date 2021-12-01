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

typedef enum
{
	ST_NOMINAL,
	ST_ALL_WHITE,
	ST_TWO_LINE,
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
const int photo_pins[] = {A9, A13, A11, A10, A8, A14, A12};
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
	if (sum < 2100)
		return ST_ALL_WHITE;
	if (sum < 2700)
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
	case ST_TWO_LINE:
		return "2 lines";
	default:
		return "Bruh";
	}
}

// calculates a pseudoposition from photoresistor readings. This is raw and uncalibrated, thus unusable without real_position().
pseudoposition_t pseudoposition_calc()
{
	float position_raw = 0.0;
	float weights[7];
	int sum = 0;
	const float min_blacks[] = {400, 170, 400, 290, 380, 330, 310};

	float black_strengths[7] = {};
	for (int i = 0; i < 7; i++)
	{
		float readout = 1024.0 - ((float)analogRead(photo_pins[i]));
		//Serial.print(readout);
		//Serial.print(" \t");
		float node = (float)(i - 3);
		float weight = node * readout;
		position_raw += weight;
		weights[i] = weight;
		sum += readout;
		black_strengths[i] = (readout > min_blacks[i]);

		Serial.print(black_strengths[i]);
		Serial.print(" \t");
	}
	Serial.println();

	/*
	float best_strength = 0;
	int best_strength_position = 0;
	for (int i = 1; i < 6; i++) {
		float strength = black_strengths[i-1] + black_strengths[i] + black_strengths[i+1];
		if (strength > best_strength) {
			best_strength = strength;
			best_strength_position = i;
		}
	}
	float right_extent = .5;
	for (int i = best_strength_position + 1; i < 7; i++) {
		right_extent += black_strengths[i];
		if (black_strengths[i] < 0.8) {
			break;
		}
	}
	float left_extent = .5;
	for (int i = best_strength_position - 1; i >= 0; i++) {
		left_extent += black_strengths[i];
		if (black_strengths[i] < 0.8) {
			break;
		}
	}
	float left_value = best_strength_position - left_extent;
	float right_value = best_strength_position + right_extent;
	*/
	//Serial.print(left_value);
	//Serial.print("\t");
	//Serial.print(right_value);


	//Serial.println(position_raw);

	Serial.println();

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
	// y = 1.1193e-8 x^3 + 0.000015x^2 + 0.009167x+2.0907
	// [0.011193,15.,9167.,2.0907e6]
	float x = pseudoposition;

	/*
	const float scale = 1000000;
	const float c1 = 0.011193;
	const float c2 = 15;
	const float c3 = 9167;
	const float c4 = 2.0907;
	*/

	//0.0027693048879217*a^(3)+6.5471960667387*a^(2)+8572.0268775523*a+3305136.1067313
	const float scale = 1000000;
	const float c1 = 0.002769;
	const float c2 = 6.5472;
	const float c3 = 8572.03;
	const float c4 = 3.30514;

	float t1 = c1 * (x * x * x);
	float t2 = c2 * (x * x);
	float t3 = c3 * (x);

	float poly = t1 + t2 + t3;

	return poly / scale + c4;
}

float leftSpeedValid, rightSpeedValid;
line_state_t run_state;
int state_lock = 0;

#define ST_LOCKED(state) (state_lock != 0 && run_state == (state))

// get PID controller output given error and speed, using the PID constants from the LUT
void pid_step(float error, float *speed, int *leftSpeed, int *rightSpeed)
{

	if (fabs(error) > 10.0)
	{
		*speed = 0;
		error = abs(error) / error * 10.0;
	}

	float output = 0;
	pid_t terms = pid_lerp(*speed);
	output += terms.p * error;
	output += terms.i * err_acc;
	output += terms.d * (error - previous_pos);
	err_acc += error;

	if (err_acc > terms.r)
	{
		err_acc = terms.r;
	}
	else if (err_acc < -terms.r)
	{
		err_acc = -terms.r;
	}

	*speed += 0.02;
	*speed -= abs(0.2 * terms.d * (error - previous_pos)) * 3.0 / 255.0;
	if (*speed > 2)
		*speed = 2;
	if (*speed < 0)
		*speed = 0;

	// OVERRIDE!
	*speed = 1;

	if (output > 0)
	{
		*leftSpeed = terms.s;
		*rightSpeed = terms.s - (int)output;
		*rightSpeed = *rightSpeed < -255 ? -255 : *rightSpeed;
	}
	else
	{
		*leftSpeed = terms.s + (int)output;
		*rightSpeed = terms.s;
		*leftSpeed = *leftSpeed < -255 ? -255 : *leftSpeed;
	}

	if (state == ST_NOMINAL)
	{
		leftSpeedValid = *leftSpeed;
		rightSpeedValid = *rightSpeed;
		if (run_state == ST_ALL_BLACK && fabs(error) < 0.5)
			run_state = ST_NOMINAL;
		else if (run_state == ST_ALL_BLACK)
			goto all_blk;
	}
	else if (state == ST_ALL_BLACK || ST_LOCKED(ST_ALL_BLACK))
	//else
	{
	all_blk:;
	/*
		run_state = ST_ALL_BLACK;
		if (!ST_LOCKED(ST_ALL_BLACK))
			state_lock = 20;
		if (state_lock == 1 && fabs(error) > 0.5)
			state_lock = 8;
		*leftSpeed = abs(leftSpeedValid) / leftSpeedValid * 30;
		*rightSpeed = abs(rightSpeedValid) / rightSpeedValid * 30;
		*speed = 0;
		*/
	}
	
	else if (state == ST_ALL_WHITE && !ST_LOCKED(ST_ALL_BLACK))
	{
		run_state = ST_ALL_WHITE;
		// 	//if(!ST_LOCKED(ST_ALL_WHITE)) state_lock = 20;
		// 	//if(state_lock == 1 && fabs(error) > 0.5) state_lock = 8;
		//*leftSpeed = abs(leftSpeedValid) / leftSpeedValid * 40 - 80;
		//*rightSpeed = abs(rightSpeedValid) / rightSpeedValid * 40 - 80;
		*leftSpeed = 0;
		*rightSpeed = 0;
		*speed = 0;
	}


	//Serial.println(state_str(state));

	// if (state_lock)
	// 	state_lock--;
	// else
	// 	run_state = ST_NOMINAL;

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
	pid_step(real_position(pseudoposition_calc().position), &speed, &leftSpeed, &rightSpeed);
	if (leftSpeed > 0)
	{
		Motor2->setSpeed(leftSpeed);
		Motor2->run(FORWARD);
	}
	else
	{
		Motor2->setSpeed(-leftSpeed);
		Motor2->run(BACKWARD);
	}

	if (rightSpeed > 0)
	{
		Motor4->setSpeed(rightSpeed);
		Motor4->run(FORWARD);
	}
	else
	{
		Motor4->setSpeed(-rightSpeed);
		Motor4->run(BACKWARD);
	}

	// if(run_state == ST_ALL_BLACK) delay(250);
	// if(run_state == ST_ALL_WHITE) delay(250);
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

	// if (bad_state != bad_state_prev)
	// {
	// 	//Serial.print("Bad state? ");
	// 	//Serial.println(bad_state);
	// }
	// bad_state_prev = bad_state;
}