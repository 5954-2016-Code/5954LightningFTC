package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ericw on 11/6/2016.
 */

public class LargeBallLiftSystem {

    public DcMotor ArmDrive1 = null,
            ArmDrive2 = null;

    public Servo sLiftClaw = null;

    private static final double servoMidpoint = 0.5,
                                servoMovement = 0.5;

    public void init(HardwareMap HWMap) {
        ArmDrive1 = HWMap.dcMotor.get("mArm1");
        ArmDrive2 = HWMap.dcMotor.get("mArm2");

        sLiftClaw = HWMap.servo.get("sLiftClaw");
    }

    public void armDrive1(double Power)
    {
        Power = Range.clip(-Power, -1, 1);
        ArmDrive1.setPower(Power);
    }
    public void armDrive2(double Power)
    {
        Power = Range.clip(Power, -1, 1);
        ArmDrive2.setPower(Power);
    }

    public void operateClaw(boolean open, boolean close){
        sLiftClaw.setPosition(servoMidpoint + (open?servoMovement:0) - (close?servoMovement:0));

    }
}
