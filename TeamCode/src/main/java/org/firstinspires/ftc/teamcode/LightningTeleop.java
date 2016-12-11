package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop - comp", group = "5954")
public class LightningTeleop extends LightningFunctions {

    //Main Teleop Loop that reads the gamepads and performs the
    //driver functions as described below
    @Override
    public void loop() {
        //Move the robot using the arcade drive system
        this.DriveSystem.ArcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        //Operate the arm lift
        this.BallLift.armDrive1(gamepad2.left_stick_y);
        //Use gamepad2 x and b buttons to manually operate the button push functions
        this.ButtonPush.TeleopButtonPush(gamepad2.x, gamepad2.b);

        //Use gamepad 2 a and y buttons to pull in/release the large cap ball
        this.BallLift.operateClaw(gamepad2.a,gamepad2.y);

        //Use gamepad2 left bumper/left trigger to pull in/push out balls
        this.BallManagement.Intake(gamepad2.left_bumper, (gamepad2.left_trigger >0.2));

        //Operate the drawer slide lift system with gamepad2 right bumper(lift)/right trigger(lower)
        this.BallManagement.Lift(gamepad2.right_bumper, (gamepad2.right_trigger >0.2));

        //gamepad2 right bumper also spins the ball shooting wheels to shoot the balls
        //into the center vortex
        if (gamepad2.right_bumper) {
            this.BallShooter.ShootBall();
        }
        else{
            //don't spin the shooting wheels if the button is not pressed
            this.BallShooter.StopShooter();
        }

        //Driver enhanced function
        //1. Pushes out the guide servo to drive along the wall
        //2. Drives the robot forward
        //3. Looks for the appropriate colored beacon:
        //blue for gamepad1 x, red for gamepad1 b
        //4. pushes the beacon button when the appropriate color is found

        //Perform the function only while the x or b buttons are held in
        //otherwise cancel the function
        while(gamepad1.x == true || gamepad1.b == true) {
            //When the button is first found
            if (beaconFindStarted == false) {
                //Push out the guide wheel with a servo
                ButtonPush.FrontPushOut();
                beaconFindStarted = true;
            }

            //Push the blue button when the blue color sensor reads above 14
            if ((gamepad1.x == true) && (ButtonPush.csPushR.blue() > 14)) {
                //Push out the "Beacon Button Bopper"
                ButtonPush.RearPushOut();
                //Stop the drive motor
                DriveSystem.ArcadeDrive(0, 0);
            }
            //Press red button
            else if ((gamepad1.b == true) && (ButtonPush.csPushR.red() > 14)) {
                //Push out the "Beacon Button Bopper"
                ButtonPush.RearPushOut();
                //Stop the drive motor
                DriveSystem.ArcadeDrive(0, 0);
            } else {
                //Drive Forward if the desired color has not been found yet
                DriveSystem.ArcadeDrive(-.20, 0);
            }
        }

        //Stop the button press driver feature when the button is released
        if (gamepad1.x == false && gamepad1.b == false && beaconFindStarted == true)
        {
            //Retract the wall guide servo
            ButtonPush.FrontPushIn();
            //Retract the "Beacon Button Bopper"
            ButtonPush.RearPushIn();
            //Stop the drive motor
            DriveSystem.ArcadeDrive(0,0);
            beaconFindStarted = false;
        }

        //Output the current red/blue color sensor readings to the telemetry display for testing
        telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
        telemetry.update();
    }
}
