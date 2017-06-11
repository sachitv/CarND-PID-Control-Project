# Project Writeup

## Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected?

After reading the document described by the reviewer [here](https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf), the explanation of the P, I and D components are as follows:

**P** : This is the most important component as this controls the movement (or other control scheme) of the vehicle with direct proportionality to that of the cross track error. This value is usually larger than the other two since it plays the largest part in the control.

**I** : This controls the control scheme with respect to the total accumulated error of the system. It's important in order to ensure stability of the vehicle. Since this value can grow over time, it usually has a much smaller value. It becomes really handy when we are close to zero error and we would like to control this so that we can get there.

**D** : This controls the control scheme with respect to the change in error between the current and previous readings. It has a smaller value than P but a larger value than I. As quoted from the document : "The job of the derivative is to predict the future value for the error, and then make the speed act accordingly. For example, if it thinks it will overshoot, it will slow it down."

## Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

This is a rather interesting question. I did this in a very different way from a lot of other folks. The truth is - there is no final hyperparameters, my system is continuously tuning them.

Initially, I initialize the system with the parameters given as 

Kp,     Ki      Kd  
0.01,   0.001,  1.0 - For Steering  
0.05,   0.005,  0.9 - For Acceleration 

And then I allow twiddle to tweak the steering for the first 200 frames before tweaking acceleration begins. I attempt to control the acceleration with a target speed of 30 on straights and a minimum of 25 around corners depending on the amount of turning required. More turning = lower speed and vice versa.

The initialization values were chosen with a bit of hit and miss. They seemed to provide good initial results so I used them.