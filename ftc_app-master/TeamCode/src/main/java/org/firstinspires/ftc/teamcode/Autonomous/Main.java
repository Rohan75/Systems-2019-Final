package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Main extends LinearOpMode {

    Hware hardware = new Hware(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        hardware.mapDrive();

        waitForStart();

    }

}
