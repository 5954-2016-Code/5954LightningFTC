package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class LightningAutoBlue extends LinearOpMode {

    DriveSystemBase DriveSystem = new DriveSystemBase();
    BallManagementSystem BallManagement = new BallManagementSystem();
    BallShooterSystem BallShooter = new BallShooterSystem();
    ButtonPushSystem ButtonPush = new ButtonPushSystem();
    LargeBallLiftSystem BallLift = new LargeBallLiftSystem();

    // Chassis Sensors
    public ColorSensor csChasis = null;

    private boolean first_run = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSystem.init(hardwareMap);
        BallManagement.init(hardwareMap);
        BallShooter.init(hardwareMap);
        ButtonPush.init(hardwareMap);
        BallLift.init(hardwareMap);

        DriveSystem.init_gyro();

        // Chassis Sensor Init
        csChasis = hardwareMap.colorSensor.get("csChasis");

        first_run = false;

        while (!DriveSystem.imuChasis.isGyroCalibrated())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        DriveSystem.runWithEncoder();

        while (!isStarted()) {
            DriveSystem.angles   = DriveSystem.imuChasis.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData(">", "Robot Heading = %.1f", DriveSystem.angleDegrees(DriveSystem.angles.angleUnit, DriveSystem.angles.firstAngle));
            telemetry.addData(">", "Robot Heading = %s", new Func<String>() {
                @Override public String value() {
                    return DriveSystem.formatAngle(DriveSystem.angles.angleUnit, DriveSystem.angles.firstAngle);
                }
            });
            telemetry.update();
            idle();
        }
        DriveSystem.resetIMU_Position_Integration();


    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * DriveSystem.COUNTS_PER_INCH);
            newLeftTarget = DriveSystem.getLeftMotorEncoderVal()+ moveCounts;
            newRightTarget = DriveSystem.getRightMotorEncoderVal() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            DriveSystem.setLeftTargetPos(newLeftTarget);
            DriveSystem.setRightTargetPos(newRightTarget);
            DriveSystem.runLeftMotorToPos();
            DriveSystem.runRightMotorToPos();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            DriveSystem.ArcadeDrive(speed,0);

            // keep looping while we are still active, and BOTH Motors are running.
            while (opModeIsActive() &&
                    (DriveSystem.mDriveL1.isBusy() && DriveSystem.mDriveR1.isBusy())) {

                // adjust relative speed based on heading error.
                error = DriveSystem.getError(angle);
                steer = DriveSystem.getSteer(error, DriveSystem.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                double speedcheck = 0;

                if (speed+steer > 1.0)
                    speedcheck /= (speed+steer);
                else
                    speedcheck = speed;

                DriveSystem.ArcadeDrive(speedcheck,steer);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      DriveSystem.getLeftMotorEncoderVal(),
                        DriveSystem.getRightMotorEncoderVal());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            DriveSystem.ArcadeDrive(0,0);

            // Turn off RUN_TO_POSITION
            DriveSystem.runWithEncoder();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            DriveSystem.onHeading(speed, angle, DriveSystem.P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        DriveSystem.ArcadeDrive(0,0);
    }
}
