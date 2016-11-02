package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 11/2/2016.
 */

public class ButtonPushSystem {
    // Button Push System
    public Servo sPushL = null,
            sPushR = null;

    public ColorSensor csPushL = null,
            csPushR = null;

    public void init(HardwareMap HWMap) {
        // Button Push System Init
        sPushL = HWMap.servo.get("sPushL");
        sPushR = HWMap.servo.get("sPushR");
        csPushL = HWMap.colorSensor.get("csPushL");
        csPushR = HWMap.colorSensor.get("csPushR");
    }

}
