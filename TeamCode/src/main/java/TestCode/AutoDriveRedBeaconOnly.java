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

//Drive to the Red Beacons and attempt to press the red beacon buttons
//Modified to work with Adafruit IMU
@Autonomous(name="Red Only Beacon: Auto Drive", group="5954")
//@Disabled
public class AutoDriveRedBeaconOnly extends LightningAutonomousBaseOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        initGyro = false;
        initHardware(); //Performs all off the initialization+gyro calibration in the base class

        //Push the front wall guide out using the front servo
        ButtonPush.FrontPushOut();

        //Currently avoiding the gyro here because it can disturb the color sensor
        //gyroDrive(DRIVE_SPEED, 40, 0.0);  // Drive FWD
        //Drive forward for the designated inches
        encoderDrive(.55, 39.5, 39.5, 5);
        Thread.sleep(250);

        //Right Turn -- Note: Left and Right motors appear to be swapped for turning
        Motors.leftMotor.setPower(.25);
        Motors.rightMotor.setPower(.7);
        Motors.leftMotor2.setPower(.25);
        Motors.rightMotor2.setPower(.7);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(575);

        //bias towards the left a little bit
        Motors.leftMotor.setPower(.3);
        Motors.rightMotor.setPower(.25);
        Motors.leftMotor2.setPower(.3);
        Motors.rightMotor2.setPower(.25);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(2000);

        //Try stopping the gyro to avoid any conflicts
        //on the i2c bus with the color sensors
        //TODO: enable the following line if gyro drive function is needed
        //imu.stopAccelerationIntegration();
        while (ButtonPush.csPushR.red() > 200)
        {
            idle();
        }
        Thread.sleep(100);

        Motors.leftMotor.setPower(-.27);
        Motors.rightMotor.setPower(-.20);
        Motors.leftMotor2.setPower(-.27);
        Motors.rightMotor2.setPower(-.20);

        //Red Team
        telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());

        //Keep driving until the desired color level is found
        while(ButtonPush.csPushR.red() < 14)
        {
            //telemetry.addData("FColor", "%3d:%3d", csChasis.red(), csChasis.green() , csChasis.blue());
            telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
            telemetry.update();
            //Thread.sleep(2);
            idle();
        }

        //Stop the drive motors
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        //Push button on beacon
        ButtonPush.RearPushOut();

        Thread.sleep(1500);
        ButtonPush.RearPushIn();
        //ButtonPush.FrontPushIn();
        Thread.sleep(500);

        //bias to the left a little bit
        Motors.leftMotor.setPower(.20);
        Motors.rightMotor.setPower(.16);
        Motors.leftMotor2.setPower(.20);
        Motors.rightMotor2.setPower(.16);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(1500);

        //Red Team
        //telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
        while(ButtonPush.csPushR.red() < 14)
        {
            //telemetry.addData("FColor", "%3d:%3d", csChasis.red(), csChasis.green() , csChasis.blue());
            telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
            telemetry.update();
            //Thread.sleep(2);
            idle();
        }
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        //Push button on beacon
        ButtonPush.RearPushOut();

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(1500);
        ButtonPush.RearPushIn();
        ButtonPush.FrontPushIn();

//        // Start the logging of measured acceleration
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//        Thread.sleep(100);

        Motors.leftMotor.setPower(-.25);
        Motors.rightMotor.setPower(-.7);
        Motors.leftMotor2.setPower(-.25);
        Motors.rightMotor2.setPower(-.7);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(250);

        Motors.leftMotor.setPower(-.5);
        Motors.rightMotor.setPower(-.5);
        Motors.leftMotor2.setPower(-.5);
        Motors.rightMotor2.setPower(-.5);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(4000);

        //Stop the motors
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
