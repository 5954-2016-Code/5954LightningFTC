package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 11/2/2016.
 */


public class ButtonPushSystem {
    public enum BeaconColor {Red, Blue, Other}

    // Button Push System
    public Servo    sPushF = null,
                    sPushR = null;

    public ColorSensor  csPushR = null;

    static final double downPosition = .75f,
                        upPosition =  -.75f;

    public void init(HardwareMap HWMap) {
        // Button Push System Init
        sPushF = HWMap.servo.get("sPushF");
        sPushR = HWMap.servo.get("sPushR");
        sPushR.setDirection(Servo.Direction.REVERSE);
        csPushR = HWMap.colorSensor.get("csPushR");
        csPushR.setI2cAddress(I2cAddr.create8bit(0x70) );
        csPushR.enableLed(false);
    }

    public BeaconColor PollRearSensor(){
        return (csPushR.blue() > csPushR.red() ? BeaconColor.Blue : BeaconColor.Red);
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
