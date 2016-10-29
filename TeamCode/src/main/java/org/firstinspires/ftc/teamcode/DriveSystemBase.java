package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ericw on 10/29/2016.
 */

public class DriveSystemBase {
    // Drive System
    public DcMotor mDriveL1 = null,
            mDriveL2 = null,
            mDriveR1 = null,
            mDriveR2 = null;

    public DriveSystemBase(){
    }

    public void init(HardwareMap HWMap) {
        // Drive System Init
        mDriveL1 = HWMap.dcMotor.get("mDriveL1");
        mDriveL2 = HWMap.dcMotor.get("mDriveL2");
        mDriveR1 = HWMap.dcMotor.get("mDriveR1");
        mDriveR2 = HWMap.dcMotor.get("mDriveR2");
    }

    private double deadzone(double input){
        double dArea = 1.0;
        return (input > dArea ? dArea : (input < -dArea ? -dArea : input));
    }

    private void driveSystem(double left, double right){
        left = deadzone(left);
        right = deadzone(right);
        mDriveL1.setPower(left);
        mDriveL2.setPower(left);
        mDriveR1.setPower(right);
        mDriveR2.setPower(right);
    }

    public void ArcadeDrive(double ForwardPower, double TurnPower){
        driveSystem(TurnPower - ForwardPower,
                TurnPower + ForwardPower);
    }
}
