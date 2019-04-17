
/**
 * Team:    Syosset Syborgs
 * ID:      10696
 *
 * Final TeleOp
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele", group="TeleOp")
public class FinalTeleOp extends OpMode {

    // Drive System
    private static DcMotor FL, FR, BL, BR;
    private double RT, LT, RY, LY;

    //Lift System
    private DcMotor actuator;

    //Intake System
    private DcMotor shoulder, elbow;
    private CRServo intake;
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // Variables for reversing driving controls
    private boolean backwardsDrive = false;
    private ElapsedTime etime = new ElapsedTime();

    @Override
    public void init() {

        // Map drive system motors
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse one wheel side for mecanum
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop immediately
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Actuator mapping
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake System
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(CRServo.class, "intake");
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        modernRoboticsI2cGyro.calibrate();
    }

    @Override
    public void loop() {

        // Controller inputs
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;
        LY = -gamepad1.left_stick_y;
        RY = -gamepad1.right_stick_y;

        // Switch mode
        if (gamepad1.y) {
            backwardsDrive = !backwardsDrive;
            if (backwardsDrive) {
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
                FL.setDirection(DcMotorSimple.Direction.FORWARD);
                BL.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                FR.setDirection(DcMotorSimple.Direction.FORWARD);
                BR.setDirection(DcMotorSimple.Direction.FORWARD);
                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            etime.reset();
            while(etime.time() < .3); // Slow the tick; prevent spamming
        }

        // Tell user what side is front
        telemetry.addData("Mode: ", (backwardsDrive ? "arm front" : "wires front"));

        // Reverse inputs if on backwards mode
        if (backwardsDrive) {
            RY = -gamepad1.left_stick_y;
            LY = -gamepad1.right_stick_y;
        }

        // Mecanum wheel movement
        FL.setPower(LY + RT - LT);
        BL.setPower(LY - RT + LT);
        FR.setPower(RY + LT - RT);
        BR.setPower(RY - LT + RT);


        // Actuator control
        if (gamepad1.a) {
            actuator.setPower(1);
        } else if (gamepad1.b) {
            actuator.setPower(-1);
        } else {
            actuator.setPower(0);
        }

        // Reving - joke; like in the movies
        if (gamepad1.x) {
            FL.setPower(1);
            FR.setPower(1);
            BL.setPower(-1);
            BR.setPower(-1);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

        // Control the arm with the joysticks
        shoulder.setPower(-gamepad2.left_stick_y);
        elbow.setPower(-gamepad2.right_stick_y/1.5);

        // Move intake using "A" & "B"
        if (gamepad2.b) {
            intake.setPower(1);
        } else if (gamepad2.a) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // Small movements using Arm Controller's "dpad"
        if (gamepad2.dpad_up) {
            FL.setPower(.4);
            FR.setPower(.4);
            BL.setPower(.4);
            BR.setPower(.4);
        } else if (gamepad2.dpad_down) {
            FL.setPower(-.4);
            FR.setPower(-.4);
            BL.setPower(-.4);
            BR.setPower(-.4);
        } else if (gamepad2.dpad_left) {
            FL.setPower(-.4);
            FR.setPower(.4);
            BL.setPower(-.4);
            BR.setPower(.4);
        } else if (gamepad2.dpad_right) {
            FL.setPower(.4);
            FR.setPower(-.4);
            BL.setPower(.4);
            BR.setPower(-.4);
        }

        // Gyroscope to maintain intake 90 degrees to ground
        float angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        if (gamepad2.x) {
            if (angle < -4) {
                telemetry.addData("going right", 0);
                intake.setPower(-.3);
            } else if (angle > 4) {
                telemetry.addData("going left", 0);
                intake.setPower(.3);

            }
        }
    }
}