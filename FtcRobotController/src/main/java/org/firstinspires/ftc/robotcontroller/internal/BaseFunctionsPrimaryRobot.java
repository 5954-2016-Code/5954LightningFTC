package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    DcMotor LeftDriveMotor1, LeftDriveMotor2,
            RightDriveMotor1, RightDriveMotor2,
            LeftShootMotor, RightShootMotor;

    CRServo BallSuck,
            BallConveyor;

    ColorSensor rgbSensor;

    public Time Timer;

    private DigitalChannel
            BallSensor,
            IRDetector;

    @Override
    public void init() {
        //DC MOTORS
        //Drive Right
        RightDriveMotor1 = hardwareMap.dcMotor.get("Right_Drive_Motor1");
        RightDriveMotor2 = hardwareMap.dcMotor.get("Right_Drive_Motor2");

        //Drive Left
        LeftDriveMotor1 = hardwareMap.dcMotor.get("Left_Drive_Motor1");
        LeftDriveMotor2 = hardwareMap.dcMotor.get("Left_Drive_Motor2");

        //Shooter NeveRest Motors
        LeftShootMotor = hardwareMap.dcMotor.get("Left_Shooter");
        RightShootMotor = hardwareMap.dcMotor.get("Right_Shooter");

        //Arm Extend Motors
        //ExtendBottom = hardwareMap.dcMotor.get("Extend_Bottom");
        //ExtendTop = hardwareMap.dcMotor.get("Extend_Top");

        //CONTINUOUS ROTATION SERVOS
        BallConveyor = hardwareMap.crservo.get("Ball_Convey");
        BallSuck = hardwareMap.crservo.get("Ball_Suck");

        //SENSORS
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

    public void ShootBall(boolean Button)
    {
        double shootingPower = 0;
        if (Button == true)
        {
            shootingPower = 0.75;
        }
        LeftShootMotor.setPower(-shootingPower);
        RightShootMotor.setPower(shootingPower);
    }

    public void BallSystem(double PowerForward, double PowerBackwards){

        if (PowerForward > 0){
            BallSuck.setPower(1);
            BallConveyor.setPower(1);
        }else if(PowerBackwards > 0){
            BallSuck.setPower(-1);
            BallConveyor.setPower(-1);
        }else{
            BallSuck.setPower(0);
            BallConveyor.setPower(0);
        }
    }

}
