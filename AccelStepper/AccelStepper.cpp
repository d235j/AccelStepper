// AccelStepper.cpp
//
// Copyright (C) 2009 Mike McCauley
// $Id: AccelStepper.cpp,v 1.9 2012/09/28 22:41:19 mikem Exp mikem $

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
    // Dont do anything unless we actually have a speed
    if (_speed == 0.0f)
	return false;

    unsigned long time = micros();
    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    unsigned long nextStepTime = _lastStepTime + _stepInterval;
    if (   ((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime)))
	|| ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))

    {
	if (_speed > 0.0f)
	{
	    // Clockwise
	    _currentPos += 1;
	}
	else if (_speed < 0.0f)
	{
	    // Anticlockwise  
	    _currentPos -= 1;
	}
	step(_currentPos & 0x7); // Bottom 3 bits (same as mod 8, but works with + and - numbers) 

	_lastStepTime = time;
	return true;
    }
    else
    {
	return false;
    }
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
    _targetPos = _currentPos = position;
    computeNewSpeed(); // Expect speed of 0
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
    float requiredSpeed;
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    if (distanceTo == 0)
	return 0.0f; // We're there

    // sqrSpeed is the signed square of _speed.
    float sqrSpeed = sq(_speed);
    if (_speed < 0.0)
      sqrSpeed = -sqrSpeed;
    float twoa = 2.0 * _acceleration;

    // Ensure we dont get massive acceleration when _speed is very small
    // _sqrt_twoa is precomputed when _acceleration is set
    float delta_speed = fabs(_acceleration / _speed);
    if (delta_speed > _sqrt_twoa)
	delta_speed = _sqrt_twoa;
    
    // if v^^2/2as is the the left of target, we will arrive at 0 speed too far -ve, need to accelerate clockwise
    if ((sqrSpeed / twoa) < distanceTo)
    {
	// Accelerate clockwise
	// Need to accelerate in clockwise direction
	requiredSpeed = _speed + delta_speed;
	if (requiredSpeed > _maxSpeed)
	    requiredSpeed = _maxSpeed;
    }
    else
    {
	// Decelerate clockwise, accelerate anticlockwise
	// Need to accelerate in clockwise direction
	requiredSpeed = _speed - delta_speed;
	if (requiredSpeed < -_maxSpeed)
	    requiredSpeed = -_maxSpeed;
    }
    
//    Serial.println(requiredSpeed);
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

AccelStepper::AccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    _interface = interface;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = pin1;
    _pin[1] = pin2;
    _pin[2] = pin3;
    _pin[3] = pin4;
    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
    enableOutputs();
}

AccelStepper::AccelStepper(void (*forward)(), void (*backward)())
{
    _interface = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = 0;
    _pin[1] = 0;
    _pin[2] = 0;
    _pin[3] = 0;
    _forward = forward;
    _backward = backward;
    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
}

void AccelStepper::setMaxSpeed(float speed)
{
    _maxSpeed = speed;
    computeNewSpeed();
}

void AccelStepper::setAcceleration(float acceleration)
{
    _acceleration = acceleration;
    _sqrt_twoa = sqrt(2.0f * _acceleration);
    computeNewSpeed();
}

void AccelStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;

    if ((speed > 0.0f) && (speed > _maxSpeed))
        _speed = _maxSpeed;
    else if ((speed < 0.0f) && (speed < -_maxSpeed))
        _speed = -_maxSpeed;
    else
        _speed = speed;
    _stepInterval = fabs(1000000.0 / _speed);
}

float AccelStepper::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepper::step(uint8_t step)
{
    switch (_interface)
    {
        case FUNCTION:
            step0();
            break;

	case DRIVER:
	    step1(step);
	    break;
    
	case FULL2WIRE:
	    step2(step);
	    break;
    
	case FULL4WIRE:
	    step4(step);
	    break;  

	case HALF4WIRE:
	    step8(step);
	    break;  
    }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step0()
{
  if (_speed > 0)
    _forward();
  else
    _backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step1(uint8_t step)
{
    // _pin[0] is step, _pin[1] is direction
    setOutputPins((_speed > 0) ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time 
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    setOutputPins((_speed > 0) ? 0b10 : 0b00); // step LOW

}

// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step2(uint8_t step)
{
    switch (step & 0x3)
    {
	case 0: /* 01 */
	    setOutputPins(0b10);
//	    digitalWrite(_pin[0], LOW);
//	    digitalWrite(_pin[1], HIGH);
	    break;

	case 1: /* 11 */
	    setOutputPins(0b11);
//	    digitalWrite(_pin[0], HIGH);
//	    digitalWrite(_pin[1], HIGH);
	    break;

	case 2: /* 10 */
	    setOutputPins(0b01);
//	    digitalWrite(_pin[0], HIGH);
//	    digitalWrite(_pin[1], LOW);
	    break;

	case 3: /* 00 */
	    setOutputPins(0b00);
//	    digitalWrite(_pin[0], LOW);
//	    digitalWrite(_pin[1], LOW);
	    break;
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step4(uint8_t step)
{
    switch (step & 0x3)
    {
	case 0:    // 1010
	    setOutputPins(0b0101);
//	    digitalWrite(_pin[0], HIGH);
//	    digitalWrite(_pin[1], LOW);
//	    digitalWrite(_pin[2], HIGH);
//	    digitalWrite(_pin[3], LOW);
	    break;

	case 1:    // 0110
	    setOutputPins(0b0110);
//	    digitalWrite(_pin[0], LOW);
//	    digitalWrite(_pin[1], HIGH);
//	    digitalWrite(_pin[2], HIGH);
//	    digitalWrite(_pin[3], LOW);
	    break;

	case 2:    //0101
	    setOutputPins(0b1010);
//	    digitalWrite(_pin[0], LOW);
//	    digitalWrite(_pin[1], HIGH);
//	    digitalWrite(_pin[2], LOW);
//	    digitalWrite(_pin[3], HIGH);
	    break;

	case 3:    //1001
	    setOutputPins(0b1001);
//	    digitalWrite(_pin[0], HIGH);
//	    digitalWrite(_pin[1], LOW);
//	    digitalWrite(_pin[2], LOW);
//	    digitalWrite(_pin[3], HIGH);
	    break;
    }
}


// 4 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step8(uint8_t step)
{
    switch (step & 0x7)
    {
	case 0:    // 1000
	    setOutputPins(0b0001);
//            digitalWrite(_pin[0], HIGH);
//            digitalWrite(_pin[1], LOW);
//            digitalWrite(_pin[2], LOW);
//            digitalWrite(_pin[3], LOW);
            break;
	    
        case 1:    // 1010
	    setOutputPins(0b0101);
//            digitalWrite(_pin[0], HIGH);
//            digitalWrite(_pin[1], LOW);
//            digitalWrite(_pin[2], HIGH);
//            digitalWrite(_pin[3], LOW);
            break;
	    
	case 2:    // 0010
	    setOutputPins(0b0100);
//            digitalWrite(_pin[0], LOW);
//            digitalWrite(_pin[1], LOW);
//            digitalWrite(_pin[2], HIGH);
//            digitalWrite(_pin[3], LOW);
            break;
	    
        case 3:    // 0110
	    setOutputPins(0b0110);
//            digitalWrite(_pin[0], LOW);
//            digitalWrite(_pin[1], HIGH);
//            digitalWrite(_pin[2], HIGH);
//            digitalWrite(_pin[3], LOW);
            break;
	    
	case 4:    // 0100
	    setOutputPins(0b0010);
//            digitalWrite(_pin[0], LOW);
//            digitalWrite(_pin[1], HIGH);
//            digitalWrite(_pin[2], LOW);
//            digitalWrite(_pin[3], LOW);
            break;
	    
        case 5:    //0101
	    setOutputPins(0b1010);
//            digitalWrite(_pin[0], LOW);
//            digitalWrite(_pin[1], HIGH);
//            digitalWrite(_pin[2], LOW);
//            digitalWrite(_pin[3], HIGH);
            break;
	    
	case 6:    // 0001
	    setOutputPins(0b1000);
//            digitalWrite(_pin[0], LOW);
//            digitalWrite(_pin[1], LOW);
//            digitalWrite(_pin[2], LOW);
//            digitalWrite(_pin[3], HIGH);
            break;
	    
        case 7:    //1001
	    setOutputPins(0b1001);
//            digitalWrite(_pin[0], HIGH);
//            digitalWrite(_pin[1], LOW);
//            digitalWrite(_pin[2], LOW);
//            digitalWrite(_pin[3], HIGH);
            break;
    }
}
    
// Prevents power consumption on the outputs
void    AccelStepper::disableOutputs()
{   
    if (! _interface) return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
        digitalWrite(_enablePin, LOW ^ _enableInverted);
}

void    AccelStepper::enableOutputs()
{
    if (! _interface) 
	return;

    pinMode(_pin[0], OUTPUT);
    pinMode(_pin[1], OUTPUT);
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
    {
        pinMode(_pin[2], OUTPUT);
        pinMode(_pin[3], OUTPUT);
    }

    if (_enablePin != 0xff)
    {
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
        pinMode(_enablePin, OUTPUT);
    }
}

void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void AccelStepper::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
        pinMode(_enablePin, OUTPUT);
    }
}

void AccelStepper::setPinsInverted(bool direction, bool step, bool enable)
{
    _pinInverted[0] = step;
    _pinInverted[1] = direction;
    _enableInverted = enable;
}


// Blocks until the target position is reached
void AccelStepper::runToPosition()
{
    while (run())
	;
}

boolean AccelStepper::runSpeedToPosition()
{
    if (_targetPos >_currentPos)
	_speed = fabs(_speed);
    else
	_speed = -fabs(_speed);
    return _targetPos!=_currentPos ? runSpeed() : false;
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}
