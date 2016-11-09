package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by ericw on 10/29/2016.
 */

enum autonSteps {Step1, Step2, Step3, Step4, Step5, Step6, Step7, Step8, Step9, Step10,
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

//        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData(">", "Calibrating Gyro");    //
//        telemetry.update();

        // make sure the gyro is calibrated before continuing
        //timeoutTimer = new ElapsedTime();
        //timeoutTimer.reset();

        //timeout if the gyro calibration takes more than 10 seconds
//        while (DriveSystem.imuChasis.isGyroCalibrated())  {
//            try {
//                Thread.sleep(50);
//            }
//            catch (InterruptedException ex) {
//                //Do nothing? or do we actually want to "return"
//            }
//            Thread.yield(); //Yield to other threads while we wait for the gyro calibration to complete
//        }
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();

        first_run = false;
        autonomousStep = autonSteps.Step1;
    }

    @Override
    public void init_loop(){
//        if (DriveSystem.imuChasis.isGyroCalibrated()) {
//            telemetry.addData(">", "Robot Heading = %.1f", DriveSystem.getGyroAngle());
//            telemetry.update();
//        }
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
