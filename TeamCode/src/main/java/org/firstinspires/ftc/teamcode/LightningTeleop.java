package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ericw on 11/4/2016.
 */

@TeleOp(name = "Teleop - comp", group = "Test")

public class LightningTeleop extends LightningFunctions {

    @Override
    public void loop() {
        this.DriveSystem.ArcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        this.BallLift.armDrive1(gamepad2.left_stick_y);
        this.ButtonPush.TeleopButtonPush(gamepad2.x, gamepad2.b);
        this.BallLift.operateClaw(gamepad2.a,gamepad2.y);
        this.BallManagement.Intake(gamepad2.left_bumper, (gamepad2.left_trigger >0.2));
        this.BallManagement.Lift(gamepad2.right_bumper, (gamepad2.right_trigger >0.2));

        if (gamepad2.right_bumper) {
            this.BallShooter.ShootBall();
        }
        else{
            this.BallShooter.StopShooter();
        }

        //BallManagement.isBallDetected();

        telemetry.addData(">", "Distance = %.1f", BallManagement.currentDistanceValue);
        telemetry.update();
    }
}
