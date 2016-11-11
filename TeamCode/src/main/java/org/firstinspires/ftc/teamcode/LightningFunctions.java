package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by ericw on 10/29/2016.
 */

enum autonSteps {Init1, Init2, Step1, Step2, Step3, Step4, Step5, Step6, Step7, Step8, Step9, Step10,
                 Step11, Step12, Step13, Step14, Step15, Step16, Step17, Step18, Step19, Step20};

public class LightningFunctions extends OpMode {
    public autonSteps autonomousStep;

    DriveSystemBase DriveSystem = new DriveSystemBase();
    BallManagementSystem BallManagement = new BallManagementSystem();
    BallShooterSystem BallShooter = new BallShooterSystem();
    ButtonPushSystem ButtonPush = new ButtonPushSystem();
    LargeBallLiftSystem BallLift = new LargeBallLiftSystem();

    // Chassis Sensors
    public ColorSensor csChasis = null;

    // Misc Properties
    private ElapsedTime period  = new ElapsedTime();

    private boolean first_run = false;

    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    private ElapsedTime timeoutTimer;

    /********************************************************************
     * Overridden Functions
     */

    @Override
    public void init() {
        DriveSystem.init(hardwareMap);
        BallManagement.init(hardwareMap);
        BallShooter.init(hardwareMap);
        ButtonPush.init(hardwareMap);
        BallLift.init(hardwareMap);

        // Chassis Sensor Init
        csChasis = hardwareMap.colorSensor.get("csChasis");

        first_run = false;
        autonomousStep = autonSteps.Init1;
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
    }

    public double angleDegrees(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }


}
