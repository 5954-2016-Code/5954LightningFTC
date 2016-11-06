package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by ericw on 10/29/2016.
 */

public class DriveSystemBase {
    // Drive System
    public DcMotor mDriveL1 = null,
            mDriveL2 = null,
            mDriveR1 = null,
            mDriveR2 = null;


    public BNO055IMU imuChasis = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1120; //(cpr): 1120 (280 rises of Channel A) // eg: TETRIX Motor Encoder: 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1;   //2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 0.5;  //Original: 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.0375; //Original: .1    // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.07; //Original: 0.15;     // Larger is more responsive, but also less stable

    public DriveSystemBase(){
    }

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

    private void resetIMU_Position_Integration()
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
}
