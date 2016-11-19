package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//TODO: This was an attempt to perform Autonomous using standard OpMode
//For now it takes too long to calibrate the gyro
@Autonomous(name = "Autonomous-comp", group = "Test")
@Disabled
public class LightningAutonomuous extends LightningFunctions {

    VisionSystem vision = new VisionSystem();
    private static ButtonPushSystem.BeaconColor teamColor = ButtonPushSystem.BeaconColor.Red;

    @Override
    public void init()
    {
        super.init(); //perform all of the inits of the parent class
        vision.init();
        vision.startTracking(); //I guess this could be placed in the VisionSystem init?

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        this.DriveSystem.imuChasis.initialize(DriveSystem.parameters);

        // make sure the gyro is calibrated before continuing
        //timeoutTimer = new ElapsedTime();
        //timeoutTimer.reset();

//        while (!DriveSystem.imuChasis.isGyroCalibrated())  {
//            try {
//                Thread.sleep(50);
//            }
//            catch (InterruptedException ex) {
//                //Do nothing? or do we actually want to "return"
//            }
//            //Thread.yield(); //Yield to other threads while we wait for the gyro calibration to complete
//        }
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();
    }

    @Override
    public void init_loop()
    {
//        if (autonomousStep == autonSteps.Init1) {
//            //Comment these if Gyro won't initialize
//            this.DriveSystem.imuChasis.initialize(DriveSystem.parameters);
//            autonomousStep = autonSteps.Init2;
//        }
//        else if (autonomousStep == autonSteps.Init2) {
//            // Start the logging of measured acceleration
//            this.DriveSystem.imuChasis.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//            autonomousStep = autonSteps.Step1;
//        }
        Thread.yield();
        if (DriveSystem.imuChasis.isGyroCalibrated()) {
            telemetry.addData(">", "Robot Heading = %.1f", DriveSystem.getGyroAngle());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        //Update and display position data to the selected teamColor image
        //if it is actively targeted
        //Change to tempColor.BLUE for blue team
        if (vision.isTargetLocated(teamColor))
        {
            //This updates the location data from the Motors to the target image
            vision.updateLastLocation(teamColor);
            telemetry.addData("Pos", vision.lastLocationToString());
            telemetry.update();
        }

        //Sample of getting angles and distances from Motors to target image
        //TODO: We need to figure out which of these to use to get Motors to the beacon
        if (vision.isTargetLocated(teamColor) && vision.lastLocation != null)
        {
            float x_Angle = vision.get_xAngle();
            float y_Angle = vision.get_yAngle();
            float z_Angle = vision.get_zAngle();
            float x_mm = vision.get_xMillimeters();
            float y_mm = vision.get_yMillimeters();
            float z_mm = vision.get_zMillimeters();
        }



        //Chasis Color Sensor -- Does alpha correspond to white?
        if (this.csChasis.alpha() >=12)
        {
            //found white line?
        }



        switch(autonomousStep)
        {
            case Step1:
                //Set the GyroDrive goals
                DriveSystem.setGyroDrive(DriveSystemBase.DRIVE_SPEED, 12, 0.0);
                autonomousStep = autonSteps.Step2;
                break;
            case Step2:
                //Continue looping the runGyroDrive method until the goal has been reached
                if (DriveSystem.runGyroDrive()==false)
                {
                    autonomousStep = autonSteps.Step3;
                }
                break;
            case Step3:
                //Continue looping until the following turn is complete
                if (DriveSystem.gyroTurn(DriveSystemBase.TURN_SPEED, -45.0)==false)
                {
                    autonomousStep = autonSteps.Step4;
                }
                break;
            case Step4:
                //Reset Timer -- holds a turn for x seconds
                 DriveSystem.startGyroHold(); //Start a timer
                 autonomousStep = autonSteps.Step5;
                break;
            case Step5:
                //Hold turn until the timer expires
                if (DriveSystem.gyroHold(DriveSystemBase.TURN_SPEED, -45.0, 0.5) == false)
                {
                    autonomousStep = autonSteps.Step6;
                }
                break;
            default:
                break;
        }

        if (DriveSystem.driveByGyroActive)
        {
            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  DriveSystem.error, DriveSystem.steer);
            telemetry.addData("Target",  "%7d:%7d",      DriveSystem.newLeftTarget,  DriveSystem.newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      DriveSystem.mDriveL1.getCurrentPosition(),
                    DriveSystem.mDriveR1.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  DriveSystem.leftSpeed, DriveSystem.rightSpeed);
            telemetry.update();
        }
        else if (DriveSystem.turnByGyroActive)
        {
            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", DriveSystem.angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", DriveSystem.error, DriveSystem.steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", DriveSystem.leftSpeed, DriveSystem.rightSpeed);
            telemetry.update();
        }
    }
}
