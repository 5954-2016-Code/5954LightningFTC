package TestCode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PIDController {
    public double kP=0;
    public double kI = 0;
    public double kD = 0;
    public double Error = 0;
    public double iError = 0;
    public double lastError = 0;
    public double SetPoint = 0;

    static double maxIntegratorValue = 5.0; //What is a good maximum?
    public double secondsSinceLastSample = 0;

    boolean uP = false, uI = false, uD = false, firstRun = true;
    //Timer eTimer;
    ElapsedTime eTimer;

    PIDController(double kPi, double kIi, double kDi){
        if (kPi==0)uP = false;
        else {
            uP = true;
            kP=kPi;
        }

        if (kIi==0)uI = false;
        else {
            uI = true;
            kI = kIi;
        }

        if (kDi==0)uD = false;
        else {
            uD = true;
            kD = kDi;
        }
        SetPoint = 0;
        firstRun = true;
        //eTimer = new Timer();
        eTimer = new ElapsedTime();

    }

    public void setSetpoint(double newSetpoint)
    {
        SetPoint = newSetpoint;
    }

    public double getSetpoint()
    {
        return SetPoint;
    }

    private void calc_Error(double input){
        lastError = Error;
        Error = SetPoint - input;
        if (firstRun){
            eTimer.reset();
        }
        else {
            secondsSinceLastSample = eTimer.seconds();
            eTimer.reset();
        }
    }

    private double calcPError(double input){
        return kP * Error;
    }

    //Biased for seconds between samples
    private double calcIError(double input){
        iError += (Error * secondsSinceLastSample);
        coerce(iError, -maxIntegratorValue, maxIntegratorValue);
        return kI * iError;
    }

    //Bias for seconds between samples
    private double calcDError(double input){
        return kD * ((lastError - Error) / secondsSinceLastSample);
    }

    public double getOutput(double input){
        calc_Error(input);
        return (uP?calcPError(input):0) + (uI?calcIError(input):0) + (uD?calcDError(input):0);
    }


    //From https://github.com/dicarlo236/2015-Robot/blob/master/src/org/usfirst/frc/team236/robot/PID.java
    /**
     * Constrains a number between min and max. Like the LabVIEW
     * "Coerce/In-range" VI.
     *
     * @param a
     * @param min
     * @param max
     * @return
     */
    public static double coerce(double a, double min, double max) {
        return Math.min(max, Math.max(a, min));
    }
}