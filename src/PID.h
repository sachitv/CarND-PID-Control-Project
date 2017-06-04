#ifndef PID_H
#define PID_H

class PID
{
	enum class TwiddleStage
	{
		ZeroRuns,
		FirstRun,
		SecondRun
	};
	enum class TwiddlerParam
	{
		P,
		I,
		D
	};
public:
	/*
	* Errors
	*/
	double p_error;
	double i_error;
	double d_error;

	/*
	* Coefficients
	*/
	double Kp;
	double Ki;
	double Kd;

	/*
	 * Other Stuff
	 */
	double lastCTE;
	double const tolerance;
	double lastdpP;
	double lastdpI;
	double lastdpD;
	double bestErr;
	double totalError;
	int numDatas;


	TwiddleStage twiddleStage;
	TwiddlerParam twiddlerParam;



	/*
	* Constructor
	*/
	PID();

	/*
	* Destructor.
	*/
	virtual ~PID();

	/*
	* Initialize PID.
	*/
	void Init( double inKp, double inKi, double inKd );

	/*
	* Update the PID error variables given cross track error.
	*/
	void UpdateError( double cte );

	/*
	* Calculate the total PID error.
	*/
	double GetSteering() const;

	/*
	 * Recalculate Parameters
	 */
	void Twiddle();
};

#endif /* PID_H */
