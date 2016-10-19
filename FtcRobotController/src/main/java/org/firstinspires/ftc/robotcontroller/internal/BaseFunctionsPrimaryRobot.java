package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Time;

/**
 * @author jwthane
 * @author spzproductions
 * @author bradenthane
 */
public class BaseFunctionsPrimaryRobot extends OpMode {

    // DC Motors
    DcMotor LeftDriveMotor1, LeftDriveMotor2,
            RightDriveMotor1, RightDriveMotor2,
            LeftShootMotor, RightShootMotor,
            ArmMotor;

           //ExtendMotor
    Servo BallSuck, BallConveyor;

    public Time Timer;
    ColorSensor rgbSensor;

    private DigitalChannel BallSensor,
            IRDetector;

    @Override
    public void init() {
        LeftDriveMotor1 = hardwareMap.dcMotor.get("Left_Drive_Motor1");
        RightDriveMotor1 = hardwareMap.dcMotor.get("Right_Drive_Motor1");
        LeftDriveMotor2 = hardwareMap.dcMotor.get("Left_Drive_Motor2");
        RightDriveMotor2 = hardwareMap.dcMotor.get("Right_Drive_Motor2");
        LeftShootMotor = hardwareMap.dcMotor.get("Left_Shooter");
        RightShootMotor = hardwareMap.dcMotor.get("Right_Shooter");
        BallConveyor = hardwareMap.servo.get("Ball_Convey");
        BallSuck = hardwareMap.servo.get("Ball_Suck");

//        ArmMotor = hardwareMap.dcMotor.get("Arm_Motor");
        //ExtendBottom = hardwareMap.dcMotor.get("Extend_Bottom");
        //ExtendTop = hardwareMap.dcMotor.get("Extend_Top");

        //BallSensor = hardwareMap.digitalChannel.get("Ball_Sensor");
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
        LeftDriveMotor1.setPower(left);
        LeftDriveMotor2.setPower(left);
        RightDriveMotor1.setPower(right);
        RightDriveMotor2.setPower(right);
    }

    public void ArcadeDrive(double ForwardPower, double TurnPower){
        driveSystem(TurnPower - ForwardPower,
                TurnPower + ForwardPower);
    }

    public void ShootBall(boolean AButtonValue)
    {
        double shootingPower = 0;
        if (AButtonValue == true)
        {
            shootingPower = 0.5;
        }
        LeftShootMotor.setPower(-shootingPower);
        RightShootMotor.setPower(shootingPower);
    }

    public void BallSystem(double PowerForward, double PowerBackwards){

        if (PowerForward > 0){
            BallSuck.setDirection(Servo.Direction.FORWARD);
            BallConveyor.setDirection(Servo.Direction.FORWARD);
        }else if(PowerBackwards > 0){
            BallSuck.setDirection(Servo.Direction.REVERSE);
            BallConveyor.setDirection(Servo.Direction.REVERSE);
        }
    }

}
