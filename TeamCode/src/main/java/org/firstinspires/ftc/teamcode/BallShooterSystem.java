package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ericw on 11/2/2016.
 */

public class BallShooterSystem {
    // Ball Shooter System
    public DcMotor  mShooterL = null,
                    mShooterR = null;

    BallShooterSystem(){

    }

    public void init(HardwareMap HWMap) {

        // Ball Shooter System Init
        mShooterL = HWMap.dcMotor.get("mShooterL");
        mShooterR = HWMap.dcMotor.get("mShooterL");
    }
}
