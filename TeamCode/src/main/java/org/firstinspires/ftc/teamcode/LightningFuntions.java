package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * Created by ericw on 10/29/2016.
 */

public class LightningFuntions extends OpMode {
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

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (DriveSystem.imuChasis.isGyroCalibrated())  {
            //These 2 lines work in LinearOpMode but
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException ex) {
                return; //If the sleep is interrupted by another thread then just quit?
            }
            Thread.yield(); //Yield to other threads while we wait for the gyro calibration to complete
        }
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        first_run = false;
    }

    @Override
    public void init_loop(){
        if (DriveSystem.imuChasis.isGyroCalibrated()) {
            telemetry.addData(">", "Robot Heading = %.1f", DriveSystem.getGyroAngle());
            telemetry.update();
        }
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
