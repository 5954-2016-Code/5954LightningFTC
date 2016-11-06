package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 11/2/2016.
 */

enum Color {Red, Blue, Other}

public class ButtonPushSystem {
    // Button Push System
    public Servo    sPushF = null,
                    sPushR = null;

    public ColorSensor  csPushR = null;

    static final double upPosition = 1.0f,
                        downPosition =  0.5f;

    public void init(HardwareMap HWMap) {
        // Button Push System Init
        sPushF = HWMap.servo.get("sPushF");
        sPushR = HWMap.servo.get("sPushR");
        csPushR = HWMap.colorSensor.get("csPushR");
    }

    public Color PoleRearSensor(){
        return (csPushR.blue() > csPushR.red() ? Color.Blue : Color.Red);
    }

    public void FrontPushOut(){
        sPushF.setPosition(downPosition);
    }

    public  void FrontPushIn(){
        sPushF.setPosition(upPosition);
    }

    public void RearPushOut(){
        sPushR.setPosition(downPosition);
    }

    public void RearPushIn(){
        sPushR.setPosition(upPosition);
    }

    public void TeleopButtonPush(boolean FrontButton, boolean RearButton){
        if (FrontButton){
            FrontPushOut();
        }
        else{
            FrontPushIn();
        }

        if (RearButton){
            RearPushOut();
        }
        else{
            RearPushIn();
        }
    }

}
