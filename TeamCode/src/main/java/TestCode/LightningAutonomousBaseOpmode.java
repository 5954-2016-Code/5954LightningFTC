package TestCode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
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

import static android.R.attr.gravity;

//This class is the basis for the Autonomous opmodes
//It initializes the hardware and has provides methods to
//drive the robot by gyro or by encoder without gyro
//Works with Adafruit Inertial Measurement Unit
public class LightningAutonomousBaseOpmode extends LinearOpMode {

    /* Declare OpMode members. */
    public LightningDrive Motors = new LightningDrive();
    public BallManagementSystem BallManagement = new BallManagementSystem();
    public BallShooterSystem BallShooter = new BallShooterSystem();
    public ButtonPushSystem ButtonPush = new ButtonPushSystem();
    public LargeBallLiftSystem BallLift = new LargeBallLiftSystem();
    //public ColorSensor csChasis = null;

    // The Adafruit IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation newAngles;
    public ElapsedTime holdTimer = new ElapsedTime();

    public boolean dontDriveLeftWheels = false;
    public boolean dontDriveRightWheels = false;

    public boolean initGyro = false;

    Acceleration gravity;
    //Output counts per revolution of Output Shaft
    static final double     COUNTS_PER_MOTOR_REV    = 1120; //(cpr): 1120 (280 rises of Channel A) // eg: TETRIX Motor Encoder: 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1;   //2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific Motors drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 1; //0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 2;  //Original: 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.005; //Original: .1    // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.03; //Original: 0.15;     // Larger is more responsive, but also less stable

    //This code should reset the IMU position to 0 and start polling the sensor in a thread
    //for the current heading
    public void resetIMU_Position_Integration()throws InterruptedException
    {
         imu.stopAccelerationIntegration();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //To give the imu thread a chance to get going we should do some
        //idle(); calls here but can only do them in our runOpMode()

        double checkDegrees = 180;
        //Give gyro a chance to get going (re-zeroed) after reset
        holdTimer.reset();
        while((Math.abs(checkDegrees)>0.5) && (holdTimer.seconds()<0.1)) {
            idle(); //Yield to other threads including gyro threads
            newAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            checkDegrees = angleDegrees(newAngles.angleUnit, newAngles.firstAngle);
        }
    }

    public void initHardware() throws InterruptedException
    {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        Motors.init(hardwareMap);
        BallManagement.init(hardwareMap);
        BallShooter.init(hardwareMap);
        ButtonPush.init(hardwareMap);
        BallLift.init(hardwareMap);
        BallManagement.Intake(false, false);
        BallManagement.Lift(false, false);
        ButtonPush.FrontPushIn();
        ButtonPush.RearPushIn();

        // Chassis Sensor Init
        //csChasis = hardwareMap.colorSensor.get("csChasis");
        //testColorSensor = (LightningColorSensor)hardwareMap.colorSensor.get("csChasis");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Added the initGyro variable so that we can avoid the 6-7 second gyro initialization
        //if the gyro is not being used.
        if (initGyro == true) {
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }

        Motors.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure the Motors it stationary, then reset the encoders and calibrate the gyro.
        Motors.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            if (initGyro == true) {
                newAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                telemetry.addData(">", "Robot Heading = %.1f", angleDegrees(newAngles.angleUnit, newAngles.firstAngle));
            }
            telemetry.addData("BColor", "%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.blue());
            telemetry.addData(">", "Robot Ready.");
            telemetry.update();
            idle();
        }

        if (initGyro == true) {
            //This really isn't necessary but should help if the Motors were bumped during calibration
            resetIMU_Position_Integration();
        }


        Motors.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BallLift.armDrive1(-1);
        Thread.sleep(250);
        BallLift.armDrive1(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //start with initHardware();
        //Add the actual autonomous robot actions in each
        //Extended class
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
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
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = Motors.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = Motors.rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            Motors.leftMotor.setTargetPosition(newLeftTarget);
            Motors.rightMotor.setTargetPosition(newRightTarget);

            Motors.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motors.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            Motors.leftMotor.setPower(speed);
            Motors.rightMotor.setPower(speed);
            Motors.leftMotor2.setPower(speed);
            Motors.rightMotor2.setPower(speed);

            // keep looping while we are still active, and BOTH Motors are running.
            while (opModeIsActive() &&
                    (Motors.leftMotor.isBusy() && Motors.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                Motors.leftMotor.setPower(leftSpeed);
                Motors.rightMotor.setPower(rightSpeed);
                Motors.leftMotor2.setPower(leftSpeed);
                Motors.rightMotor2.setPower(rightSpeed);

                if (distance > 0)
                {
                    if (Motors.leftMotor.getCurrentPosition() >= newLeftTarget)
                    {
                        // Stop all motion;
                        Motors.leftMotor.setPower(0);
                        Motors.rightMotor.setPower(0);
                        Motors.leftMotor2.setPower(0);
                        Motors.rightMotor2.setPower(0);

                        // Turn off RUN_TO_POSITION
                        Motors.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Motors.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        break;
                    }
                }

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("ActualL",  "%7d:%7d",      Motors.leftMotor.getCurrentPosition(),
                        Motors.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                telemetry.addData("FColor", "%3d:%3d:%3d", csChasis.red(), csChasis.green() + csChasis.blue());
//                telemetry.addData("BColor", "%3d:%3d:%3d", ButtonPush.csPushR.red(), ButtonPush.csPushR.green() + ButtonPush.csPushR.blue());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            Motors.leftMotor.setPower(0);
            Motors.rightMotor.setPower(0);
            Motors.leftMotor2.setPower(0);
            Motors.rightMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            Motors.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motors.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            Motors.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            Motors.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public void gyroTurn (  double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }

        //If we tried a turn that wasn't driving one side of Motors
        //then be sure that we clear the variable so both sides drive
        //next time
        dontDriveRightWheels = false;
        dontDriveLeftWheels = false;
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold( double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimerGyroHold = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimerGyroHold.reset();
        while (opModeIsActive() && (holdTimerGyroHold.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        Motors.leftMotor.setPower(0);
        Motors.rightMotor.setPower(0);
        Motors.leftMotor2.setPower(0);
        Motors.rightMotor2.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to Motors.
        // dontDrive Left/Right Wheels to try turn near wall for beacon
        // only autonomous mode
        if (dontDriveLeftWheels == false) {
            Motors.leftMotor.setPower(leftSpeed);
            Motors.leftMotor2.setPower(leftSpeed);
        }

        if (dontDriveRightWheels == false) {
            Motors.rightMotor.setPower(rightSpeed);
            Motors.rightMotor2.setPower(rightSpeed);
        }

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the Motors's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the Motors's frame of reference
     *          +ve error means the Motors should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        newAngles  = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        robotError = targetAngle + AngleUnit.DEGREES.fromUnit(newAngles.angleUnit, newAngles.firstAngle);
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in Motors relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    public double angleDegrees(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        //return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = Motors.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = Motors.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Motors.leftMotor.setTargetPosition(newLeftTarget);
            Motors.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            Motors.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motors.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            holdTimer.reset();
            //Motors.leftMotor.setPower(Math.abs(speed));
            //Motors.rightMotor.setPower(Math.abs(speed));
            Motors.leftMotor.setPower(Math.abs(speed));
            Motors.leftMotor2.setPower(Math.abs(speed));
            Motors.rightMotor.setPower(Math.abs(speed));
            Motors.rightMotor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (holdTimer.seconds() < timeoutS) &&
                    /*Motors.leftMotor.getCurrentPosition() < newLeftTarget)*/
                    (Motors.leftMotor.isBusy() && Motors.rightMotor.isBusy()))
                {
                //if (distance > 0)
                //{
                    if ((Motors.leftMotor.getCurrentPosition() >= newLeftTarget)
                            || (Motors.rightMotor.getCurrentPosition() >= newRightTarget))
                    {
                        // Stop all motion;
                        Motors.leftMotor.setPower(0);
                        Motors.rightMotor.setPower(0);
                        Motors.leftMotor2.setPower(0);
                        Motors.rightMotor2.setPower(0);

                        // Turn off RUN_TO_POSITION
                        Motors.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Motors.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        break;
                    }
                //}

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        Motors.leftMotor.getCurrentPosition(),
                        Motors.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Motors.leftMotor.setPower(0);
            Motors.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            Motors.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motors.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


}
