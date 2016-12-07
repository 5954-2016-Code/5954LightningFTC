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

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the Motors.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the Motors must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the Motors is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the Motors Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Modified to work with Adafruit IMU

@Autonomous(name="Blue Only Beacon: Auto Drive", group="5954")
//@Disabled
public class AutoDriveBlueBeaconOnly extends LightningAutonomousBaseOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); //Performs all off the initialization+gyro calibration in the base class

        ButtonPush.FrontPushOut();

        //TODO: Time this out so that we don't cross the line before 10 seconds have passed

        //TODO: Test if this will work with target 180 degrees?
        //It doesn't like to drive backward with gyroDrive--it tries to turn around
        //gyroDrive(DRIVE_SPEED, -24 /*-96*/, 180);  // Drive RWD
        //

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
        Thread.sleep(500);
        //Thread.sleep(3300);

        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);
        Thread.sleep(100);
        //Thread.sleep(7000);

        Motors.leftMotor.setPower(-.7);
        Motors.rightMotor.setPower(-.25);
        Motors.leftMotor2.setPower(-.7);
        Motors.rightMotor2.setPower(-.25);
        Thread.sleep(640);

        Motors.leftMotor.setPower(-1.0);
        Motors.rightMotor.setPower(-1.0);
        Motors.leftMotor2.setPower(-1.0);
        Motors.rightMotor2.setPower(-1.0);
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
        Thread.sleep(1500);

        //Left Turn -- Note: Left and Right motors appear to be swapped for turning
        Motors.leftMotor.setPower(-.25);
        Motors.rightMotor.setPower(-.7);
        Motors.leftMotor2.setPower(-.25);
        Motors.rightMotor2.setPower(-.7);
        Thread.sleep(250);

        //bias towards the left a little bit
        Motors.leftMotor.setPower(.20);
        Motors.rightMotor.setPower(.20);
        Motors.leftMotor2.setPower(.20);
        Motors.rightMotor2.setPower(.20);

        Thread.sleep(2000);

        //Try stopping the gyro to avoid any conflicts
        //on the i2c bus with the color sensors
        imu.stopAccelerationIntegration();
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
        while(ButtonPush.csPushR.red() < 10)
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
        Motors.leftMotor.setPower(.24);
        Motors.rightMotor.setPower(.20);
        Motors.leftMotor2.setPower(.24);
        Motors.rightMotor2.setPower(.20);

        Thread.sleep(1500);

        //Red Team
        //telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
        while(ButtonPush.csPushR.red() < 10)
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
        ButtonPush.FrontPushIn();

        Motors.leftMotor.setPower(.25);
        Motors.rightMotor.setPower(.8);
        Motors.leftMotor2.setPower(.25);
        Motors.rightMotor2.setPower(.8);
        Thread.sleep(300);

        Motors.leftMotor.setPower(.55);
        Motors.rightMotor.setPower(.45);
        Motors.leftMotor2.setPower(.55);
        Motors.rightMotor2.setPower(.45);

        Thread.sleep(2000);

        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
