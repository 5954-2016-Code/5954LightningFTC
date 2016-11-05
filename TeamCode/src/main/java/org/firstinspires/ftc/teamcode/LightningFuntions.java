package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ericw on 10/29/2016.
 */

public class LightningFuntions extends OpMode {
    DriveSystemBase DriveSystem = new DriveSystemBase();
    BallManagementSystem BallManagement = new BallManagementSystem();
    BallShooterSystem BallShooter = new BallShooterSystem();
    ButtonPushSystem ButtonPush = new ButtonPushSystem();

    // Chassis Sensors
    public ColorSensor csChasis = null;

    // Misc Properties
    private ElapsedTime period  = new ElapsedTime();



    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /********************************************************************
     * Overridden Functions
     */

    @Override
    public void init() {

        DriveSystem.init(hardwareMap);
        BallManagement.init(hardwareMap);
        BallShooter.init(hardwareMap);
        ButtonPush.init(hardwareMap);

        // Chassis Sensor Init
        csChasis = hardwareMap.colorSensor.get("csChasis");

    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
    }
}
