package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ericw on 11/2/2016.
 */

public class BallManagementSystem {
    // Ball Management System
    public Servo    sBallIntake = null,
                    sBallLift = null;

    public OpticalDistanceSensor dBallIntake = null;
    public double currentDistanceValue = 0;

    ElapsedTime holdTimerIntakeHold = new ElapsedTime();

    public BallManagementSystem(){

    }

    public void init(HardwareMap HWMap){
        // Ball Management System Init
        sBallIntake = HWMap.servo.get("sBallIntake");
        sBallLift = HWMap.servo.get("sBallLift");
        dBallIntake = HWMap.opticalDistanceSensor.get("dBallIntake");
        dBallIntake.enableLed(true);
        holdTimerIntakeHold.reset();
    }

    private void IntakePower(double power){
        sBallIntake.setPosition(power/2 + 0.5f);
    }
    private void LiftPower(double power){
        sBallLift.setPosition(power/2 + 0.5f);
    }

    public void Intake(boolean In, boolean Out){
        if (In){
            IntakePower(1);
        }
        else if (Out){
            IntakePower(-1);
        }
        else {
            IntakePower(0);
        }
    }

    public void Lift(boolean Up, boolean Down){
        if (Up /*|| isBallDetected()*/){
            LiftPower(-1);
        }
        else if (Down){
            LiftPower(1);
        }
        else{
            LiftPower(0);
        }
    }

    public boolean isBallDetected()
    {
        currentDistanceValue = dBallIntake.getLightDetected();
        if (currentDistanceValue > 0.25)
        {
            holdTimerIntakeHold.reset();
            return true;
        }
        else if (holdTimerIntakeHold.seconds() < 1)
        {
            return true;
        }

        return false;
    }
}
