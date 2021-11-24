#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include <FastLED.h>
#include <math.h>

/*
TYPEDEFS
*/

// represent P, I, and D constants for the PID controller
typedef struct
{
	float p;
	float i;
	float d;
	float r;
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

/*
VARIABLE INITIALIZATIONS
*/

CRGB leds[5];
#define LEDS_PIN 6

// a LUT mapping current speed to predetermined corresponding PID constants
const pid_t speedLUT[] = {
	(pid_t){1, 2, 3}, // Speed 0
	(pid_t){
		// Speed 1 (40)
		25,
		0.05,
		2,
		8,
	},
	(pid_t){
		// Speed 2 (80)
		20,
		0.1,
		90,
		50,
	},
	(pid_t){
		// Speed 3 (120)
		65,
		0.1,
		50,
		50,
	},
	(pid_t){1, 2, 3}, // Speed 4
	(pid_t){1, 2, 3}, // Speed 4 (Duplicated)
};

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

int leftSpeed = 0, rightSpeed = 0;

float previous_pos; // previous position
float err_acc;		// accumulated error

/*
FUNCTION DEFINTIONS
*/

// lerps PID LUT to provide appropriate PID terms for a given speed
pid_t pid_lerp(float n)
{
	int whole = (int)n;
	float frac = n - whole;
	float frac_inv = 1.0 - frac;

	pid_t sl = speedLUT[whole + 1];
	pid_t sh = speedLUT[whole + 2];

	return (pid_t){
		sh.p * frac + sl.p * frac_inv,
		sh.i * frac + sl.i * frac_inv,
		sh.d * frac + sl.d * frac_inv};
}

// calculates a pseudoposition from photoresistor readings
pseudoposition_t pseudoposition_calc()
{
	float position_raw = 0.0;
	float weights[7];
	float max;

	for (int i = 0; i < 7; i++)
	{
		float readout = 1024.0 - ((float)analogRead(photo_pins[i]));
		float node = (float)(i - 3);
		float weight = node * readout;
		position_raw += weight;
		weights[i] = weight;
	}

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
float real_position(float pseudoposition)
{
	// y = 1.1193e-8 x^3 + 0.000015x^2 + 0.009167x+2.0907
	// [0.011193,15.,9167.,2.0907e6]
	float x = pseudoposition;

	const float scale = 1000000;
	const float c1 = 0.011193;
	const float c2 = 15;
	const float c3 = 9167;
	const float c4 = 2.0907;

	float t1 = c1 * (x * x * x);
	float t2 = c2 * (x * x);
	float t3 = c3 * (x);

	float poly = t1 + t2 + t3;

	return poly / scale + c4;
}

// get PID controller output given error and speed, using the PID constants from the LUT
void pid_step(float error, int speed, int *leftSpeed, int *rightSpeed)
{
	float output = 0;
	pid_t terms = speedLUT[1];
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

	if (output > 0)
	{
		*leftSpeed = speed;
		*rightSpeed = speed - (int)output;
		*rightSpeed = *rightSpeed < -255 ? -255 : *rightSpeed;
	}
	else
	{
		*leftSpeed = speed + (int)output;
		*rightSpeed = speed;
		*leftSpeed = *leftSpeed < -255 ? -255 : *leftSpeed;
	}

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
	pid_step(real_position(pseudoposition_calc().position), 40, &leftSpeed, &rightSpeed);
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
}