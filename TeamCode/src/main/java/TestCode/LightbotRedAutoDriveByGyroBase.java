package TestCode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.BallManagementSystem;
import org.firstinspires.ftc.teamcode.BallShooterSystem;
import org.firstinspires.ftc.teamcode.ButtonPushSystem;
import org.firstinspires.ftc.teamcode.LargeBallLiftSystem;

import java.util.Locale;

//Modified to work with Adafruit IMU
@Autonomous(name="Red Base: Auto Drive", group="5954")
//@Disabled
public class LightbotRedAutoDriveByGyroBase extends LightningAutonomousBaseOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        initGyro = true;
        initHardware(); //Performs all off the initialization+gyro calibration in the base class

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        gyroDrive(DRIVE_SPEED, 44+8, 0.0);  // Drive FWD
        Thread.sleep(750);

        BallShooter.ShootBall();
        Thread.sleep(500);

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < 4)) {  //2 seconds
            BallManagement.Lift(true, false);
            idle();
        }

        BallManagement.Lift(false, false);
        BallShooter.StopShooter();

        gyroDrive(DRIVE_SPEED, 8, 0.0);  // Drive FWD

        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
