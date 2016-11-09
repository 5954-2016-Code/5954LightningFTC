package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author jwthane
 * @author SPZ Productions
  */
@TeleOp(name="TeleopTestRobot", group ="5954")
@Disabled
public class teleopTestRobot extends BaseFunctionsTestRobot {

    @Override
    public void loop() {
        this.ArcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        //this.ArmPower(gamepad2.left_stick_y);
        //this.ExtendPower(gamepad2.right_stick_y);
        //this.GrabberPosition(gamepad2.right_trigger);

        //telemetry.addData("LeftDriveMotor: ", LeftDriveMotor.getDeviceName().toString());
        //this.updateTelemetry(telemetry);
        //telemetry.update();
    }
}
