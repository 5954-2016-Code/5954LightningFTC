package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ericw on 10/29/2016.
 */

public class LightningFuntions extends OpMode {
    HardwareLightning robot = new HardwareLightning();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
    }
}
