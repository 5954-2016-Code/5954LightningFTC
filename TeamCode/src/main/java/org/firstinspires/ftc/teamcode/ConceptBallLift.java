package org.firstinspires.ftc.team5954;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ericw on 10/22/2016.
 */
@TeleOp(name = "Concept: Ball Lift", group = "Concept")
public class ConceptBallLift extends OpMode {


    Servo sLiftMotor, sBallIntake;
    DcMotor mShoot1, mShoot2;

    @Override
    public void init() {
        //DC Motors
        mShoot1 = hardwareMap.dcMotor.get("shooter1");
        mShoot2 = hardwareMap.dcMotor.get("shooter2");
        mShoot2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        sLiftMotor = hardwareMap.servo.get("Lift");
        sBallIntake = hardwareMap.servo.get("Intake");
    }

    @Override
    public void loop() {
        sLiftMotor.setPosition(gamepad1.left_stick_y/2 + 0.5f);
        sBallIntake.setPosition(gamepad1.right_stick_y/2 + 0.5f);
        mShoot1.setPower(gamepad2.left_stick_y);
        mShoot2.setPower(gamepad2.left_stick_y);

    }
}
