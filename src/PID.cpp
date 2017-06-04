#include "PID.h"
#include <limits>
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

static int const MIN_STEPS = 200;

PID::PID()
		: p_error( 0.f )
		  , i_error( 0.f )
		  , d_error( 0.f )
		  , Kp( 0.f )
		  , Ki( 0.f )
		  , Kd( 0.f )
		  , lastCTE( 0.f )
		  , tolerance( 0.2f )
		  , lastdpD( 0.25f )
		  , lastdpI( 0.25f )
		  , lastdpP( 0.25f )
		  , twiddleStage( TwiddleStage::ZeroRuns )
		  , twiddlerParam( TwiddlerParam::P )
		  , bestErr( numeric_limits<double>::max())
		  , totalError( 0.f )
		  , numDatas( 0 ) {}

PID::~PID() {}

void PID::Init( double inKp, double inKi, double inKd )
{
	Kp = inKp;
	Kd = inKd;
	Ki = inKi;
}

void PID::UpdateError( double cte )
{
	p_error = cte;
	d_error = cte - lastCTE;
	i_error += cte;

	lastCTE = cte;

	numDatas += 1;

	if ( numDatas > MIN_STEPS )
	{
		totalError = (totalError * (numDatas - 1) + (cte * cte)) / numDatas;
	}
}

double PID::GetControl() const
{
	double amount = -(Kp * p_error) - (Kd * d_error) - (Ki * i_error);
	amount = max( -1.0, amount );
	amount = min( 1.0, amount );

	return amount;
}

void PID::Twiddle()
{
	double const error = totalError;
	switch ( twiddleStage )
	{
		case TwiddleStage::ZeroRuns:
		{
			switch ( twiddlerParam )
			{
				case TwiddlerParam::P:
				{
					double const sum = lastdpP + lastdpI + lastdpD;
					if ( sum > tolerance )
					{
						Kp += lastdpP;

						//Go to the next stage on the next run
						twiddleStage = TwiddleStage::FirstRun;
					}
				}
					break;
				case TwiddlerParam::I:
				{
					Ki += lastdpI;

					//Go to the next stage on the next run
					twiddleStage = TwiddleStage::FirstRun;
				}
					break;
				case TwiddlerParam::D:
				{
					Kd += lastdpD;

					//Go to the next stage on the next run
					twiddleStage = TwiddleStage::FirstRun;
				}
					break;
			}
		}
			break;
		case TwiddleStage::FirstRun:
		{
			if ( error < bestErr )
			{
				bestErr = error;
				twiddleStage = TwiddleStage::ZeroRuns;
				switch ( twiddlerParam )
				{
					case TwiddlerParam::P:
					{
						lastdpP *= 1.1;
						twiddlerParam = TwiddlerParam::I;
					}
						break;
					case TwiddlerParam::I:
					{
						lastdpI *= 1.1;
						twiddlerParam = TwiddlerParam::D;
					}
						break;
					case TwiddlerParam::D:
					{
						lastdpD *= 1.1;
						twiddlerParam = TwiddlerParam::P;
					}
						break;
				}
			}
			else
			{
				//Next stage is the second run
				twiddleStage = TwiddleStage::SecondRun;
				switch ( twiddlerParam )
				{
					case TwiddlerParam::P:
					{
						Kp -= 2 * lastdpP;
					}
						break;
					case TwiddlerParam::I:
					{
						Ki -= 2 * lastdpI;
					}
						break;
					case TwiddlerParam::D:
					{
						Kd -= 2 * lastdpD;
					}
						break;
				}
			}
		}
			break;
		case TwiddleStage::SecondRun:
		{
			twiddleStage = TwiddleStage::ZeroRuns;

			if ( error < bestErr )
			{
				bestErr = error;
				switch ( twiddlerParam )
				{
					case TwiddlerParam::P:
					{
						lastdpP *= 1.1;
						twiddlerParam = TwiddlerParam::I;
					}
						break;
					case TwiddlerParam::I:
					{
						lastdpI *= 1.1;
						twiddlerParam = TwiddlerParam::D;
					}
						break;
					case TwiddlerParam::D:
					{
						lastdpD *= 1.1;
						twiddlerParam = TwiddlerParam::P;
					}
						break;
				}
			}
			else
			{
				switch ( twiddlerParam )
				{
					case TwiddlerParam::P:
					{
						Kp += lastdpP;
						lastdpP *= 0.9f;
						twiddlerParam = TwiddlerParam::I;
					}
						break;
					case TwiddlerParam::I:
					{
						Ki += lastdpI;
						lastdpI *= 0.9f;
						twiddlerParam = TwiddlerParam::D;
					}
						break;
					case TwiddlerParam::D:
					{
						Kd += lastdpD;
						lastdpD *= 0.9f;
						twiddlerParam = TwiddlerParam::P;
					}
						break;
				}
			}
		}
	}
}

int PID::GetSteps() const
{
	return numDatas;
}

