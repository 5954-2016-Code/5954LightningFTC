package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by ericw on 11/4/2016.
 */

@Autonomous(name = "Autonomous-comp", group = "Test")
public class LightningAutonomuous extends LightningFunctions {

    @Override
    public void loop() {

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
