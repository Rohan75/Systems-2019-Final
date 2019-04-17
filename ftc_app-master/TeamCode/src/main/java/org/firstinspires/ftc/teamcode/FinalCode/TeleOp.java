package org.firstinspires.ftc.teamcode.FinalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends LinearOpMode {

    /*
     * Gamepad 1
        * Left stick y          Driving
        * Left stick x          Driving
        * Right stick y         Driving
        * Right stick x         Driving
        * A                     Raise pinion
        * B                     Lower pinion
        * X                     Switch between tank drive & pov drive
        * Y                     Reverse controls
        * Dpad up
        * Dpad down
        * Dpad left
        * Dpad right
        * Right bumper
        * Left bumper
        * Right trigger         Driving
        * Left trigger          Driving
     * Gamepad 2
        * Left stick y          Shoulder
        * Left stick x
        * Right stick y         Driving
        * Right stick x         Driving
        * A                     Intake
        * B                     Intake
        * X
        * Y
        * Dpad up
        * Dpad down
        * Dpad left
        * Dpad right
        * Right bumper          Elbow
        * Left bumper           Elbow
        * Right trigger
        * Left trigger
     */

    // Drive system
    private DcMotor FL, FR, BL, BR;

    // Sub systems
    private DcMotor lift, shoulder, elbow, intake;

    private boolean tankDrive = true;
    private boolean backwards = false;

    private int intakePower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        mapDriveSystems();
        mapSubSystems();

        waitForStart();

        while (opModeIsActive()) {

            // Driving
            if (tankDrive) {
                tankDrive();
            } else {
                povDrive();
            }
            if (gamepad1.x) {
                tankDrive = !tankDrive;
                sleep(200);
            } if (gamepad1.y) {
                backwards = !backwards;
                sleep(200);
            }

            // Lift
            lift();

            // Arm controls
            controlArm();

            // Intake power
            controlIntake();

            // Small movements
            microDrive();

            // Telemetry
            telemetry();
        }

    }

    private void controlIntake() {
        intakePower = (gamepad2.a ? (intakePower == 1 ? 0 : 1) : (gamepad2.b ? (intakePower == -1 ? 0 : -1) : intakePower));
        intake.setPower(intakePower);
    }

    private void microDrive() {
        if (gamepad2.right_stick_y != 0 && gamepad2.right_stick_x != 0) {
            FL.setPower(Range.clip(-gamepad2.right_stick_y - gamepad2.right_stick_x, -1, 1)/4);
            FR.setPower(Range.clip(-gamepad2.right_stick_y + gamepad2.right_stick_x, -1, 1)/4);
            BL.setPower(Range.clip(-gamepad2.right_stick_y - gamepad2.right_stick_x, -1, 1)/4);
            BR.setPower(Range.clip(-gamepad2.right_stick_y + gamepad2.right_stick_x, -1, 1)/4);
        }
    }

    private void controlArm() {
        shoulder.setPower(-gamepad2.left_stick_y);
        elbow.setPower((gamepad2.left_bumper ? -.8 : (gamepad2.right_bumper ? .8 : 0)));
    }

    private void lift() {
        lift.setPower((gamepad1.a ? .6 : 0) - (gamepad1.b ? .6 : 0));
    }

    private void telemetry() {
        telemetry.addData("Mode", (tankDrive ? "Tank Drive" : "POV Drive"));
        telemetry.addData("Direction", (backwards ? "Arm Forward" : "Sybo Forward"));
        telemetry.update();
    }

    private void povDrive() {
        telemetry.addData("Mode", "POV Drive");
        double forward = (backwards ? gamepad1.left_stick_y : -gamepad1.left_stick_y);
        double strafe = (backwards ? -gamepad1.left_stick_x : gamepad1.left_stick_x);
        double spin = gamepad1.right_stick_x;
        FL.setPower(forward - strafe + spin);
        FR.setPower(forward + strafe - spin);
        BL.setPower(forward + strafe + spin);
        BR.setPower(forward - strafe - spin);
    }

    private void tankDrive() {
        telemetry.addData("Mode", "POV Drive");
        double LY = (backwards ? -gamepad1.right_stick_y : -gamepad1.left_stick_y);
        double RY = (backwards ? -gamepad1.left_stick_y : -gamepad1.right_stick_y);
        double LT = (backwards ? -gamepad1.left_trigger : gamepad1.left_trigger);
        double RT = (backwards ? -gamepad1.right_trigger : gamepad1.right_trigger);
        FL.setPower(LY - LT + RT);
        BL.setPower(LY + LT - RT);
        FR.setPower(RY + LT - RT);
        BR.setPower(RY - LT + RT);
    }

    private void mapDriveSystems() {
        // Map motors
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse one side
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop immediately
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void mapSubSystems() {
        // Lift system
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

}
