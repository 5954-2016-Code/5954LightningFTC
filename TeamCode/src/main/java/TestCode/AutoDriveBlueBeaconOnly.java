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

//Drive to the Blue Beacons and attempt to press the blue beacon buttons
//Finally drive to the ramp
//Modified to work with Adafruit IMU
@Autonomous(name="Blue Only Beacon: Auto Drive", group="5954")
//@Disabled
public class AutoDriveBlueBeaconOnly extends LightningAutonomousBaseOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        initGyro = false;
        initHardware(); //Performs all off the initialization+gyro calibration in the base class

        //Push the front wall guide out using the front servo
        ButtonPush.FrontPushOut();

//Currently these few lines are for reference only in case we want to use the getRuntime() value
//        //This is a possible way to track time
//        while (this.getRuntime() < 2)
//        {
//            idle();
//        }

        //TODO: Could use Arcade Drive to bias this section
        Motors.leftMotor.setPower(-1.0);
        Motors.rightMotor.setPower(-1.0);
        Motors.leftMotor2.setPower(-1.0);
        Motors.rightMotor2.setPower(-1.0);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(500);

        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);
        Thread.sleep(100);

        Motors.leftMotor.setPower(-.7);
        Motors.rightMotor.setPower(-.25);
        Motors.leftMotor2.setPower(-.7);
        Motors.rightMotor2.setPower(-.25);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(640);

        Motors.leftMotor.setPower(-1.0);
        Motors.rightMotor.setPower(-1.0);
        Motors.leftMotor2.setPower(-1.0);
        Motors.rightMotor2.setPower(-1.0);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(2300);

        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);
        Thread.sleep(7000);

        Motors.leftMotor.setPower(-0.5);
        Motors.rightMotor.setPower(-0.7);
        Motors.leftMotor2.setPower(-0.5);
        Motors.rightMotor2.setPower(-0.7);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(1500);

        //Left Turn -- Note: Left and Right motors appear to be swapped for turning
        Motors.leftMotor.setPower(-.25);
        Motors.rightMotor.setPower(-.7);
        Motors.leftMotor2.setPower(-.25);
        Motors.rightMotor2.setPower(-.7);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(250);

        //bias towards the left a little bit
        Motors.leftMotor.setPower(.20);
        Motors.rightMotor.setPower(.20);
        Motors.leftMotor2.setPower(.20);
        Motors.rightMotor2.setPower(.20);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(2000);

        //Try stopping the gyro to avoid any conflicts
        //on the i2c bus with the color sensors
        //TODO: enable the line below if gyrodrive function is used to prevent interference with color sensor
        //imu.stopAccelerationIntegration();
        while (ButtonPush.csPushR.blue() > 200)
        {
            idle();
        }
        Thread.sleep(100);

//        Motors.leftMotor.setPower(-.25);
//        Motors.rightMotor.setPower(-.20);
//        Motors.leftMotor2.setPower(-.25);
//        Motors.rightMotor2.setPower(-.20);

        //Blue Team
        telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());

        //Keep driving until the desired color level is found
        while(ButtonPush.csPushR.blue() < 14)
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

        Thread.sleep(1500);
        ButtonPush.RearPushIn();
        //ButtonPush.FrontPushIn();
        Thread.sleep(500);

        //bias to the left a little bit
        Motors.leftMotor.setPower(.20);
        Motors.rightMotor.setPower(.16);
        Motors.leftMotor2.setPower(.20);
        Motors.rightMotor2.setPower(.16);

        Thread.sleep(1500);

        //Red Team
        //telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());

        //Keep driving until the desired color level is found
        while(ButtonPush.csPushR.blue() < 14)
        {
            telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
            telemetry.update();
            //Thread.sleep(2);
            idle();
        }

        //Stop the motors
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        //Push button on beacon
        ButtonPush.RearPushOut();
        Thread.sleep(1500);

        //Pull in the "Beacon Button Bopper" & the front wall guide wheel
        ButtonPush.RearPushIn();
        ButtonPush.FrontPushIn();

        Motors.leftMotor.setPower(.20);
        Motors.rightMotor.setPower(.8);
        Motors.leftMotor2.setPower(.20);
        Motors.rightMotor2.setPower(.8);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(350);

        Motors.leftMotor.setPower(.55);
        Motors.rightMotor.setPower(.45);
        Motors.leftMotor2.setPower(.55);
        Motors.rightMotor2.setPower(.45);

        //Cause this thread to sleep for the designated time while the motors
        //continue to drive
        Thread.sleep(2000);

        //Stop the motors
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
