// AccelStepper.cpp
//
// Copyright (C) 2009 Mike McCauley
// $Id: AccelStepper.cpp,v 1.1 2010/04/25 02:21:18 mikem Exp mikem $

#include "WProgram.h"
#include "AccelStepper.h"

void AccelStepper::moveTo(long absolute)
{
    _targetPos = absolute;
    computeNewSpeed();
}

void AccelStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current speed
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper::runSpeed()
{
    unsigned long time = millis();
  
    if (time > _lastStepTime + _stepInterval)
    {
	if (_speed > 0)
	{
	    // Clockwise
	    _currentPos += 1;
	}
	else if (_speed < 0)
	{
	    // Anticlockwise  
	    _currentPos -= 1;
	}
	step(_currentPos & 0x3); // Bottom 2 bits (same as mod 4, but works with + and - numbers) 

	_lastStepTime = time;
	return true;
    }
    else
	return false;
}

long AccelStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
    return _targetPos;
}

long AccelStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
void AccelStepper::setCurrentPosition(long position)
{
    _currentPos = position;
}

void AccelStepper::computeNewSpeed()
{
    setSpeed(desiredSpeed());
}

// Work out and return a new speed.
// Subclasses can override if they want
// Implement acceleration, deceleration and max speed
// Negative speed is anticlockwise
// This is called:
//  after each step
//  after user changes:
//   maxSpeed
//   acceleration
//   target position (relative or absolute)
float AccelStepper::desiredSpeed()
{
    long distanceTo = distanceToGo();

    // Max possible speed that can still decelerate in the available distance
    float requiredSpeed;
    if (distanceTo == 0)
	return 0.0; // Were there
    else if (distanceTo > 0) // Clockwise
	requiredSpeed = sqrt(2.0 * distanceTo * _acceleration);
    else  // Anticlockwise
	requiredSpeed = -sqrt(2.0 * -distanceTo * _acceleration);

    if (requiredSpeed > _speed)
    {
	// Need to accelerate in clockwise direction
	if (_speed == 0)
	    requiredSpeed = sqrt(2.0 * _acceleration);
	else
	    requiredSpeed = _speed + abs(_acceleration / _speed);
	if (requiredSpeed > _maxSpeed)
	    requiredSpeed = _maxSpeed;
    }
    else if (requiredSpeed < _speed)
    {
	// Need to accelerate in anticlockwise direction
	if (_speed == 0)
	    requiredSpeed = -sqrt(2.0 * _acceleration);
	else
	    requiredSpeed = _speed - abs(_acceleration / _speed);
	if (requiredSpeed < -_maxSpeed)
	    requiredSpeed = -_maxSpeed;
    }
//  Serial.println(requiredSpeed);
    return requiredSpeed;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
boolean AccelStepper::run()
{
    if (_targetPos == _currentPos)
	return false;
    
    if (runSpeed())
	computeNewSpeed();
    return true;
}

AccelStepper::AccelStepper(uint8_t pins, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    _pins = pins;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
    enableOutputs();
}

void AccelStepper::setMaxSpeed(float speed)
{
    _maxSpeed = speed;
    computeNewSpeed();
}

void AccelStepper::setAcceleration(float acceleration)
{
    _acceleration = acceleration;
    computeNewSpeed();
}

void AccelStepper::setSpeed(float speed)
{
    _speed = speed;
    _stepInterval = abs(1000.0 / _speed);
}

float AccelStepper::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepper::step(uint8_t step)
{
    switch (_pins)
    {
	case 2:
	    step2(step);
	    break;
    
	case 4:
	    step4(step);
	    break;  
    }
}

// 2 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void AccelStepper::step2(uint8_t step)
{
    switch (step)
    {
	case 0: /* 01 */
	    digitalWrite(_pin1, LOW);
	    digitalWrite(_pin2, HIGH);
	    break;

	case 1: /* 11 */
	    digitalWrite(_pin1, HIGH);
	    digitalWrite(_pin2, HIGH);
	    break;

	case 2: /* 10 */
	    digitalWrite(_pin1, HIGH);
	    digitalWrite(_pin2, LOW);
	    break;

	case 3: /* 00 */
	    digitalWrite(_pin1, LOW);
	    digitalWrite(_pin2, LOW);
	    break;
    }
}

// 4 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void AccelStepper::step4(uint8_t step)
{
    switch (step)
    {
	case 0:    // 1010
	    digitalWrite(_pin1, HIGH);
	    digitalWrite(_pin2, LOW);
	    digitalWrite(_pin3, HIGH);
	    digitalWrite(_pin4, LOW);
	    break;

	case 1:    // 0110
	    digitalWrite(_pin1, LOW);
	    digitalWrite(_pin2, HIGH);
	    digitalWrite(_pin3, HIGH);
	    digitalWrite(_pin4, LOW);
	    break;

	case 2:    //0101
	    digitalWrite(_pin1, LOW);
	    digitalWrite(_pin2, HIGH);
	    digitalWrite(_pin3, LOW);
	    digitalWrite(_pin4, HIGH);
	    break;

	case 3:    //1001
	    digitalWrite(_pin1, HIGH);
	    digitalWrite(_pin2, LOW);
	    digitalWrite(_pin3, LOW);
	    digitalWrite(_pin4, HIGH);
	    break;
    }
}


// Prevents power consumption on the outputs
void    AccelStepper::disableOutputs()
{  
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    if (_pins == 4)
    {
	digitalWrite(_pin3, LOW);
	digitalWrite(_pin4, LOW);
    }
}

void    AccelStepper::enableOutputs()
{
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    if (_pins == 4)
    {
	pinMode(_pin3, OUTPUT);
	pinMode(_pin4, OUTPUT);
    }
}

// Blocks until the target position is reached
void AccelStepper::runToPosition()
{
    while (run())
	;
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

