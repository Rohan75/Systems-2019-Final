package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift implements LinearSubSystem {

    HardwareMap hardwareMap;

    Lift(HardwareMap hardwareMap) {
        create(hardwareMap);
    }

    @Override
    public void create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
