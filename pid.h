//Source: https://gist.github.com/bradley219/5373998#file-pid-h
//Credit to bradley219 for this class
//For usage example see the above link

#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        float Kp;// -  proportional gain
        float Ki;// -  Integral gain
        float Kd;// -  derivative gain
        float dt;// -  loop interval time
        float max;// - maximum value of manipulated variable
        float min;// - minimum value of manipulated variable
		float integral;
		float pre_error;
		float Ki2;

		bool saturatedMax = false;
		bool saturatedMin = false;		
		
        PID( float dt, float max, float min, float Kp, float Kd, float Ki );

        // Returns the manipulated variable given a setpoint and current process value
        float calculate( float setpoint, float pv );
		
		void reset(float dt, float max, float min, float Kp, float Kd, float Ki );
		void setKp(float in);
		void setKi(float in);
		void setKd(float in);
		void setMax(float in);
		void setMin(float in);
		
};

#endif