package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 11/2/2016.
 */

public class BallManagementSystem {
    // Ball Management System
    public Servo    sBallIntake = null,
                    sBallLift = null;

    public OpticalDistanceSensor dBallIntake = null;
    public double currentDistanceValue = 0;

    public BallManagementSystem(){

    }

    public void init(HardwareMap HWMap){
        // Ball Management System Init
        sBallIntake = HWMap.servo.get("sBallIntake");
        sBallLift = HWMap.servo.get("sBallLift");
        dBallIntake = HWMap.opticalDistanceSensor.get("dBallIntake");
    }

    private void IntakePower(double power){
        sBallIntake.setPosition(power/2 + 0.5f);
    }
    private void LiftPower(double power){
        sBallLift.setPosition(power/2 + 0.5f);
    }

    public void Intake(boolean In, boolean Out){
        if (In || isBallDetected()){
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
        if (Up){
            LiftPower(1);
        }
        else if (Down){
            LiftPower(-1);
        }
        else{
            LiftPower(0);
        }
    }

    public boolean isBallDetected()
    {
        //getLightDetected takes the 0 to 5 volts and converts it to 0 to 1 volts
        //TODO:  Adjust the 0.1 once we get the sensor on the Motors
        currentDistanceValue = dBallIntake.getLightDetected();
        if (currentDistanceValue <= 0.1)
        {
            return true;
        }
        return false;
    }
}
