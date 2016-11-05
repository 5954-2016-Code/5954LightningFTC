package org.firstinspires.ftc.teamcode;

/**
 * Created by ericw on 11/4/2016.
 */

public class LightningTeleop extends LightningFuntions {

    @Override
    public void loop() {
        this.DriveSystem.ArcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        this.BallManagement.Intake(gamepad1.left_bumper, gamepad2.x);
        this.BallManagement.Lift(gamepad2.a, gamepad2.b);

        if (gamepad2.right_bumper) {
            this.BallShooter.ShootBall();
        }
        else{
            this.BallShooter.StopShooter();
        }
    }
}
