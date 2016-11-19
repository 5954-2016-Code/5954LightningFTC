package TestCode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PIDController {
    double kP=0,kI = 0, kD = 0;
    double Error=0, iError=0, lastError=0;
    double SetPoint=0;
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

    private void calc_Error(double input){
        lastError = Error;
        Error = SetPoint - input;
        if (firstRun){
            eTimer.reset();
        }
        else {
            iError += (Error * eTimer.seconds());
            eTimer.reset();
        }
    }

    private double calcPError(double input){
        return kP * Error;
    }

    private double calcIError(double input){
        if (firstRun){
            eTimer.reset();
        }
        else {
            iError += (Error * eTimer.seconds());
            eTimer.reset();
        }
        return kI * iError;
    }

    private double calcDError(double input){
        return kD * (lastError - Error);
    }

    public double getOutput(double input){
        calc_Error(input);
        return (uP?calcPError(input):0) + (uI?calcIError(input):0) + (uD?calcDError(input):0);
    }
}