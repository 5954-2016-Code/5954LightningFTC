package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleopPrimaryRobot", group ="5954")
@Disabled
public class teleopPrimaryRobot extends BaseFunctionsPrimaryRobot {

    @Override
    public void loop() {
        this.ArcadeDrive(gamepad1.right_stick_x, gamepad1.left_stick_y);
        //this.ArmPower(gamepad2.left_stick_y);
        this.ShootBall(gamepad2.a);
        this.BallSystem(gamepad2.right_trigger, gamepad2.left_trigger);

        //this.ExtendPower(gamepad2.right_stick_y);
        //this.GrabberPosition(gamepad2.right_trigger);

        //telemetry.addData("LeftDriveMotor: ", LeftDriveMotor.getDeviceName().toString());
        //this.updateTelemetry(telemetry);
        //telemetry.update();
    }
}
