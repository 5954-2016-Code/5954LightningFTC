package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Time;

/**
 * Created by jwthane on 10/1/2016.
 */
public class BaseFunctionsPrimaryRobot extends OpMode {

    // DC Motors
    DcMotor LeftDriveMotor,
            RightDriveMotor,
            ArmMotor,
            ExtendMotor;

    Servo RightGrabber;
    Servo LeftGrabber;

    public Time Timer;
    ColorSensor rgbSensor;

    private DigitalChannel BallSensor,
            IRDetector;

    @Override
    public void init() {
        LeftDriveMotor = hardwareMap.dcMotor.get("Left_Drive_Motor");
        RightDriveMotor = hardwareMap.dcMotor.get("Right_Drive_Motor");
        ArmMotor = hardwareMap.dcMotor.get("Arm_Motor");
        ExtendMotor = hardwareMap.dcMotor.get("Extend_Motor");

        LeftGrabber = hardwareMap.servo.get("L_Grabber");
        RightGrabber = hardwareMap.servo.get("R_Grabber");

        //BallSensor = hardwareMap.digitalChannel.get("Ball_Sensor");
        //IRDetector = hardwareMap.digitalChannel.get("IR_Detector");
        //rgbSensor = hardwareMap.colorSensor.get("ColorSensor");
    }

    @Override
    public void loop() {

    }

    private double deadzone(double input){
        double dArea = 1.0;
        return (input > dArea ? dArea : (input < -dArea ? -dArea : input));
    }

    // Drive Control
    private void driveSystem(double left, double right){
        left = deadzone(left);
        right = deadzone(right);
        LeftDriveMotor.setPower(left);
        RightDriveMotor.setPower(right);
    }

    public void ArcadeDrive(double ForwardPower, double TurnPower){
        driveSystem(TurnPower - ForwardPower,
                TurnPower + ForwardPower);
    }

    public void GrabberPosition(float RightButtonValue)
    {
        if (RightButtonValue>0)
        {
            //Open the Grabber
            LeftGrabber.setPosition(.75);
            RightGrabber.setPosition(.75);
        }
        else
        {
            //Default--Keep the Grabber Closed
            LeftGrabber.setPosition(1);
            RightGrabber.setPosition(1);
        }
   }

    public void ArmPower(double Power){
        ArmMotor.setPower(deadzone(-Power));
    }
    public void ExtendPower(double Power){
        ExtendMotor.setPower(deadzone(-Power));
    }

}
