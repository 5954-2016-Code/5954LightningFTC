package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 11/2/2016.
 */

public class BallManagementSystem {
    // Ball Management System
    public Servo    sBallIntake = null,
                    sBallLift = null;
    BallManagementSystem(){

    }

    public void init(HardwareMap HWMap){
        // Ball Management System Init
        sBallIntake = HWMap.servo.get("sBallIntake");
        sBallLift = HWMap.servo.get("sBallLift");
    }

    public void IntakePower(double power){
        sBallIntake.setPosition(power/2 + 0.5f);
    }

    public void LiftPower(double power){
        sBallLift.setPosition(power/2 + 0.5f);
    }

}
