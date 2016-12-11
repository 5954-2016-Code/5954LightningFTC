package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class DriveSystemBase {
    // Drive System
    public DcMotor mDriveL1 = null,
            mDriveL2 = null,
            mDriveR1 = null,
            mDriveR2 = null;

    public BNO055IMU imuChasis = null;
    BNO055IMU.Parameters parameters = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1120; //(cpr): 1120 (280 rises of Channel A) // eg: TETRIX Motor Encoder: 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1;   //2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific Motors drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 0.5;  //Original: 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.0375; //Original: .1    // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.07; //Original: 0.15;     // Larger is more responsive, but also less stable

    // State used for updating telemetry
    Orientation angles;

    public DriveSystemBase(){
    }

    //initialize the drive system hardware
    public void init(HardwareMap HWMap) {
        // Drive System Init
        mDriveL1 = HWMap.dcMotor.get("mDriveL1");
        mDriveL2 = HWMap.dcMotor.get("mDriveL2");
        mDriveR1 = HWMap.dcMotor.get("mDriveR1");
        mDriveR2 = HWMap.dcMotor.get("mDriveR2");
        mDriveL1.setDirection(DcMotorSimple.Direction.REVERSE);
        mDriveL2.setDirection(DcMotorSimple.Direction.REVERSE);
        imuChasis = HWMap.get(BNO055IMU.class, "imu");

    }

    public void init_gyro(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //I moved these lines below to an init in LightningAutonomous.java so they would not interfere
        //with normal teleop drive mode -- just keeping as a reference for now
        imuChasis.initialize(parameters);
        //// Start the logging of measured acceleration
        imuChasis.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        driveByGyroActive = false;
        turnByGyroActive = false;
    }

    private double deadzone(double input){
        double dArea = 1.0;
        return (input > dArea ? dArea : (input < -dArea ? -dArea : input));
    }

    private void driveSystem(double left, double right){
        left = Range.clip(left,-1.0,1.0);
        right = Range.clip(right,-1.0,1.0);
        mDriveL1.setPower(left);
        mDriveL2.setPower(left);
        mDriveR1.setPower(right);
        mDriveR2.setPower(right);
    }

    public void ArcadeDrive(double ForwardPower, double TurnPower){
        driveSystem(TurnPower - ForwardPower,
                TurnPower + ForwardPower);
    }

    public void runWithEncoder(){
        mDriveL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mDriveR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders(){
        mDriveL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mDriveR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetIMU_Position_Integration()
    {
        imuChasis.stopAccelerationIntegration();
        // Start the logging of measured acceleration
        imuChasis.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    enum DirectionSelect{left,right,both}


    boolean atEncVal(DirectionSelect Select, int val){
        switch (Select){
            case left:
                return (Math.abs(mDriveL1.getCurrentPosition()) > val);
            case right:
                return (Math.abs(mDriveR1.getCurrentPosition()) > val);
            case both:
                return (Math.abs(mDriveL1.getCurrentPosition()) > val && (Math.abs(mDriveR1.getCurrentPosition()) > val));
            default:
                return false;
        }
    }

    boolean hasEncoderReset(DcMotor check){
        return (check.getCurrentPosition() == 0);
    }

    private final double power = 0.75;
    boolean AutonDrive(int length, double ForwardPower, double LateralPower) {
        // Always reset to false condition on check
        boolean l_return = false;

        // Ensure run mode is set to run with encoders
        runWithEncoder();

        // Check if at position currently
        if (atEncVal(DirectionSelect.both, length)){
            l_return = true;
            resetEncoders();
        }
        return l_return;
    }

    public double getGyroAngle()
    {
        angles   = imuChasis.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleDegrees(angles.angleUnit, angles.firstAngle);
    }

    public double angleDegrees(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
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

    public int     newLeftTarget;
    public int     newRightTarget;
    public int     moveCounts;
    public double  max;
    public double  error;
    public double  steer;
    public double  leftSpeed;
    public double  rightSpeed;
    public double angle;
    private double speed;
    private double distance;
    public boolean driveByGyroActive;
    public boolean turnByGyroActive;

    /********************************
     * Encoder Functions
     */
    public int getLeftMotorEncoderVal(){ return mDriveL1.getCurrentPosition();}
    public int getRightMotorEncoderVal(){ return mDriveR1.getCurrentPosition();}
    public void setLeftTargetPos(int target){mDriveL1.setTargetPosition(target);}
    public void setRightTargetPos(int target){mDriveR1.setTargetPosition(target);}
    public void runLeftMotorToPos(){mDriveL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void runRightMotorToPos() {mDriveR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    public void setGyroDrive ( double desiredSpeed,
                            double desiredDistance,
                            double desiredAngle) {

        // Determine new target position, and pass to motor controller
        moveCounts = (int) (distance * COUNTS_PER_INCH);
        newLeftTarget = mDriveL1.getCurrentPosition() + moveCounts;
        newRightTarget = mDriveR1.getCurrentPosition() + moveCounts;

        //copy desiredAngle, desiredDistance & desiredSpeed for the runGyroDrive() method
        angle = desiredAngle;
        distance = desiredDistance;
        speed = desiredSpeed;

        // Set Target and Turn On RUN_TO_POSITION
        mDriveL1.setTargetPosition(newLeftTarget);
        mDriveR1.setTargetPosition(newRightTarget);

        mDriveL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mDriveR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        mDriveL1.setPower(speed);
        mDriveL2.setPower(speed);
        mDriveR1.setPower(speed);
        mDriveR2.setPower(speed);

        driveByGyroActive = true;
    }

    //Returns true when encoder position reached
    public boolean runGyroDrive()
    {
            // keep looping while we are still active, and BOTH Motors are running.
            if  (mDriveL1.isBusy() && mDriveR1.isBusy()) {
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

                mDriveL1.setPower(leftSpeed);
                mDriveL2.setPower(leftSpeed);
                mDriveR2.setPower(rightSpeed);
                mDriveR2.setPower(rightSpeed);
                return true; //true meaning the Motors is still driving
            }
            else {
                // Stop all motion;
                mDriveL1.setPower(0);
                mDriveL2.setPower(0);
                mDriveR1.setPower(0);
                mDriveR2.setPower(0);

                // Turn off RUN_TO_POSITION
                mDriveL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mDriveL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveByGyroActive = false;
            }
            return false; //false meaning the Motors has stopped
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param desiredSpeed Desired speed of turn.
     * @param desiredAngle Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public boolean gyroTurn (  double desiredSpeed, double desiredAngle) {
        angle = desiredAngle; //for telemetry display in LightningAutonomous

        // keep looping while we are still active, and not on heading.
        if (onHeading(desiredSpeed, desiredAngle, P_TURN_COEFF)) {
            turnByGyroActive = false;
            return false; //false meaning the Motors is no longer turning
        }
        else
        {
            turnByGyroActive = true;
            return true; //true meaning the Motors is still turning
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param desiredSpeed  Desired speed of turn.
     * @param desiredangle  Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param desiredHoldTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    private ElapsedTime holdTimer;
    public void startGyroHold()
    {
        holdTimer = new ElapsedTime();
        holdTimer.reset();
    }

    public boolean gyroHold( double desiredSpeed, double desiredAngle, double desiredHoldTime)
    {
        angle = desiredAngle; //for telemetry display in LightningAutonomous

        // keep looping while we have time remaining.
        if(holdTimer.time() < desiredHoldTime) {
            onHeading(desiredSpeed, desiredAngle, P_TURN_COEFF);
            turnByGyroActive = true;
            return true; //true meaning the Motors is still turning/desired time has not elapsed
        }

        // Stop all motion;
        mDriveL1.setPower(0);
        mDriveL2.setPower(0);
        mDriveR1.setPower(0);
        mDriveR2.setPower(0);
        turnByGyroActive = false;
        return false; //false meaning the Motors is done turning/desired time elapsed
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
        mDriveL1.setPower(leftSpeed);
        mDriveL2.setPower(leftSpeed);
        mDriveR1.setPower(rightSpeed);
        mDriveR2.setPower(rightSpeed);

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
        //newAngles  = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //robotError = targetAngle + AngleUnit.DEGREES.fromUnit(newAngles.angleUnit, newAngles.firstAngle);
        robotError = targetAngle + getGyroAngle();
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        //return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*********************************************************
     * Gyro Drive
     */



}
