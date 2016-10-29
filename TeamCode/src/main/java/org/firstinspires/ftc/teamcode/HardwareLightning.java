package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ericw on 10/29/2016.
 */

public class HardwareLightning {


    // Ball Management System
    public Servo    sBallIntake = null,
                    sBallLift = null;

    // Ball Shooter System
    public DcMotor  mShooterL = null,
                    mShooterR = null;

    // Button Push System
    public Servo    sPushL = null,
                    sPushR = null;

    public ColorSensor  csPushL = null,
                        csPushR = null;

    // Chassis Sensors
    public ColorSensor  csChasis = null;
    public BNO055IMU imuChasis = null;

    // Misc Properties
    private ElapsedTime period  = new ElapsedTime();


    public HardwareLightning(){

    }

    public void init(HardwareMap HWMap){


        // Ball Management System Init
        sBallIntake = HWMap.servo.get("sBallIntake");
        sBallLift = HWMap.servo.get("sBallLift");

        // Ball Shooter System Init
        mShooterL = HWMap.dcMotor.get("mShooterL");
        mShooterR = HWMap.dcMotor.get("mShooterL");

        // Button Push System Init
        sPushL = HWMap.servo.get("sPushL");
        sPushR = HWMap.servo.get("sPushR");
        csPushL = HWMap.colorSensor.get("csPushL");
        csPushR = HWMap.colorSensor.get("csPushR");

        // Chassis Sensor Init
        csChasis = HWMap.colorSensor.get("csChasis");
        imuChasis = HWMap.get(BNO055IMU.class, "imu");
    }

    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}
