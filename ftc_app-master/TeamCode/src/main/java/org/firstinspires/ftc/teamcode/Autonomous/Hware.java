package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hware {

    HardwareMap hardwareMap;

    // Drive Motors
    DcMotor FL, FR, BL, BR;

    Hware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    void mapDrive() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR = hardwareMap.get(DcMotor.class, "FR");
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BL = hardwareMap.get(DcMotor.class, "BL");
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BR = hardwareMap.get(DcMotor.class, "BR");
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
