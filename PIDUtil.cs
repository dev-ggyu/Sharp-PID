using System;

namespace Util
{
    class PIDUtil
    {

        public enum PIDControlMode
        {
            AUTOMATIC = 1,
            MANUAL = 0
        }

        public enum PIDControlDirection
        {
            DIRECT = 0,
            REVERSE = 1
        }

        public enum PIDPOnMode
        {
            P_ON_M = 0,
            P_ON_E = 1
        }

        double dispKp;              // * we'll hold on to the tuning parameters in user-entered 
        double dispKi;              //   format for display purposes
        double dispKd;              //

        double kp;                  // * (P)roportional Tuning Parameter
        double ki;                  // * (I)ntegral Tuning Parameter
        double kd;                  // * (D)erivative Tuning Parameter

        int controllerDirection;
        int pOn;

        double myInput;              // * Pointers to the Input, Output, and Setpoint variables
        double myOutput;             //   This creates a hard link between the variables and the 
        double mySetpoint;           //   PID, freeing the user from having to constantly tell us
        //   what these values are.  with pointers we'll just know.

        ulong lastTime;
        double outputSum, lastInput;

        ulong SampleTime;
        double outMin, outMax;
        bool inAuto, pOnE;

        ulong millis()
        {
            return (ulong)(DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond);
        }

        public PIDUtil(double Kp, double Ki, double Kd, int POn, int ControllerDirection)
        {
            myInput = 0;
            myOutput = 0;
            mySetpoint = 0;
            inAuto = false;

            SetOutputLimits(0, 100);                //default output limit corresponds to

            SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

            SetControllerDirection(ControllerDirection);
            SetTunings(Kp, Ki, Kd, POn);

            lastTime = millis() - SampleTime;
        }
        public PIDUtil(double Kp, double Ki, double Kd, int ControllerDirection) : this(Kp, Ki, Kd, (int)PIDPOnMode.P_ON_E, ControllerDirection)
        {

        }

        public PIDUtil(ref double Input, ref double Output, ref double Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
        {
            myOutput = Output;
            myInput = Input;
            mySetpoint = Setpoint;
            inAuto = false;

            SetOutputLimits(0, 100);				//default output limit corresponds to
												        //the arduino pwm limits

            SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

            SetControllerDirection(ControllerDirection);
            SetTunings(Kp, Ki, Kd, POn);

            lastTime = millis()-SampleTime;
        }


        public PIDUtil(ref double Input, ref double Output, ref double Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
        : this(ref Input, ref Output, ref Setpoint, Kp, Ki, Kd, (int)PIDPOnMode.P_ON_E, ControllerDirection)
        {

        }

        public bool Compute()
        {
            if (!inAuto) return false;
            ulong now = millis();
            ulong timeChange = (now - lastTime);
            //Console.WriteLine("now {0}, lastTime {1}, timechange {2}",now, lastTime, timeChange);
            if (timeChange >= SampleTime)
            {
                /*Compute all the working error variables*/
                double input = myInput;
                double error = mySetpoint - input;
                double dInput = (input - lastInput);
                outputSum += (ki * error);

                /*Add Proportional on Measurement, if P_ON_M is specified*/
                if (!pOnE) outputSum -= kp * dInput;

                if (outputSum > outMax) outputSum = outMax;
                else if (outputSum < outMin) outputSum = outMin;

                /*Add Proportional on Error, if P_ON_E is specified*/
                double output;
                if (pOnE) output = kp * error;
                else output = 0;

                /*Compute Rest of PID Output*/
                output += outputSum - kd * dInput;

                if (output > outMax) output = outMax;
                else if (output < outMin) output = outMin;
                myOutput = output;

                /*Remember some variables for next time*/
                lastInput = input;
                lastTime = now;

                //Console.WriteLine("Input:{0},output:{1},Input:{2}",myInput,myOutput,mySetpoint);
                return true;
            }
            else return false;
        }

        public void SetData(double Input, double Output, double Setpoint)
        {
            myOutput = Output;
            myInput = Input;
            mySetpoint = Setpoint;
        }
        public void SetTunings(double Kp, double Ki, double Kd, int POn)
        {
            if (Kp < 0 || Ki < 0 || Kd < 0) return;

            pOn = POn;
            pOnE = (PIDPOnMode)POn == PIDPOnMode.P_ON_E;

            dispKp = Kp; dispKi = Ki; dispKd = Kd;

            double SampleTimeInSec = ((double)SampleTime) / 1000;
            kp = Kp;
            ki = Ki * SampleTimeInSec;
            kd = Kd / SampleTimeInSec;

            if ((PIDControlDirection)controllerDirection == PIDControlDirection.REVERSE)
            {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
        }

        public void SetTunings(double Kp, double Ki, double Kd)
        {
            SetTunings(Kp, Ki, Kd, pOn);
        }

        public void SetSampleTime(int NewSampleTime)
        {
            if (NewSampleTime > 0)
            {
                double ratio = (double)NewSampleTime
                                / (double)SampleTime;
                ki *= ratio;
                kd /= ratio;
                SampleTime = (ulong)NewSampleTime;
            }
        }

        public void SetOutputLimits(double Min, double Max)
        {
            if (Min >= Max) return;
            outMin = Min;
            outMax = Max;

            if (inAuto)
            {
                if (myOutput > outMax) myOutput = outMax;
                else if (myOutput < outMin) myOutput = outMin;

                if (outputSum > outMax) outputSum = outMax;
                else if (outputSum < outMin) outputSum = outMin;
            }
        }

        public void SetMode(int Mode)
        {
            bool newAuto = ((PIDControlMode)Mode == PIDControlMode.AUTOMATIC);
            if (newAuto && !inAuto)
            {  /*we just went from manual to auto*/
                Initialize();
            }
            inAuto = newAuto;
        }

        void Initialize()
        {
            outputSum = myOutput;
            lastInput = myInput;
            if (outputSum > outMax) outputSum = outMax;
            else if (outputSum < outMin) outputSum = outMin;
        }

        public void SetControllerDirection(int Direction)
        {
            if (inAuto && Direction != controllerDirection)
            {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
            controllerDirection = Direction;
        }

        public double GetKp() { return dispKp; }
        public double GetKi() { return dispKi; }
        public double GetKd() { return dispKd; }
        public int GetMode() { return inAuto ? (int)PIDControlMode.AUTOMATIC : (int)PIDControlMode.MANUAL; }
        public int GetDirection() { return controllerDirection; }

        public ref double GetOutput()
        {
            return ref myOutput;
        }

        public ref double GetInput()
        {
            return ref myInput;
        }

        public ref double GetSetpoint()
        {
            return ref mySetpoint;
        }
    }
}
