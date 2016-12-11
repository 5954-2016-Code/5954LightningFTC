package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BallShooterSystem {
    // Ball Shooter System
    private DcMotor  mShooterL = null,
                    mShooterR = null;

    public BallShooterSystem(){

    }

    public void init(HardwareMap HWMap) {

        // Ball Shooter System Init
        mShooterL = HWMap.dcMotor.get("mShooterL");
        mShooterR = HWMap.dcMotor.get("mShooterR");
        mShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void Power(double input){
        mShooterL.setPower(input);
        mShooterR.setPower(input);
    }

    public void ShootBall(){
        Power(1.00);
    }

    public void StopShooter(){
        Power(0.0);
    }
}
