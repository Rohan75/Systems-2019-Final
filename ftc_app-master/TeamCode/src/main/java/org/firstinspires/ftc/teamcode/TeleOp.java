/*
    Syborgs 10696 TeleOp
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele", group = "TeleOp")
public class TeleOp extends OpMode {

    // Drive System
    private static DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    //Lift System
    private DcMotor actuator;
    //Intake System
    private DcMotor shoulder, elbow;
    private Servo intake;
    // Team marker
    private Servo marker;

    // Used in reversing drive controls
    private boolean backwardsDrive = false;
    private ElapsedTime etime = new ElapsedTime();

    // Used in POV/Tank drive conversion
    private boolean tank = true;

    @Override
    public void init() {
        // Map drive system motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse one left side
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop immediately
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Actuator mapping
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake System
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(Servo.class, "intake");
        marker = hardwareMap.get(Servo.class, "marker");
    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            tank = false;
        } else if (gamepad1.left_bumper) {
            tank = true;
        }

        // variables for robot movement
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStick = -gamepad1.left_stick_y;
        double rightStick = -gamepad1.right_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        if (tank) {

            // reverse motors if "y" is pressed
            if (gamepad1.y) {
                backwardsDrive = !backwardsDrive;
                if (backwardsDrive) {
                    frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                    backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                    frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                    backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                } else {
                    frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                    backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                    frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                    backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // prevent from registering event too many times
                etime.reset();
                while (etime.time() < .3) {

                }
            }

            // keep the spin direction the same
            if (backwardsDrive) {
                rightStick = -gamepad1.left_stick_y;
                leftStick = -gamepad1.right_stick_y;
            }

            // Mecanum wheel movement - tank drive with triggers for strafing
            frontLeftDrive.setPower(leftStick /*forward*/ + rightTrigger /*right strafe*/ - leftTrigger /*left strafe*/);
            backLeftDrive.setPower(leftStick - rightTrigger + leftTrigger);
            frontRightDrive.setPower(rightStick + leftTrigger - rightTrigger);
            backRightDrive.setPower(rightStick - leftTrigger + rightTrigger);
        } else {
            if (leftStick < 0 && leftStickX <.4 && leftStickX > -.4) { //Zone 1: Forward
                frontLeftDrive.setPower(leftStick);
                frontRightDrive.setPower(leftStick);
                backRightDrive.setPower(leftStick);
                backLeftDrive.setPower(leftStick);
            } else if (leftStick > .4 && leftStick < .8 && leftStickX > 0) { //Zone 2: Diagonal FR
                frontLeftDrive.setPower(leftStickX);
                backRightDrive.setPower(leftStickX);
            } else if (leftStickX > 0 && leftStick < .4 && leftStick > -.4) { //Zone 3 Strafe Right
                frontLeftDrive.setPower(leftStickX);
                frontRightDrive.setPower(-leftStickX);
                backRightDrive.setPower(-leftStickX);
                backLeftDrive.setPower(leftStickX);
            } else if (leftStick < -.4 && leftStick > -.8 && leftStickX > 0) { //Zone 4 Diagonal BR
                frontRightDrive.setPower(-leftStickX);
                backLeftDrive.setPower(-leftStickX);
            } else if (leftStick < 0 && leftStickX < .4 && leftStickX > -.4) { //Zone 5 Backwards
                frontLeftDrive.setPower(-leftStick);
                frontRightDrive.setPower(-leftStick);
                backRightDrive.setPower(-leftStick);
                backLeftDrive.setPower(-leftStick);
            } else if (leftStickX < 0 && leftStick > -.8 && leftStick < -.4) { //Zone 6: Diagonal Back Left
                frontLeftDrive.setPower(-leftStickX);
                backRightDrive.setPower(-leftStickX);
            } else if (leftStickX < 0 && leftStick > -.4 && leftStick < .4) { //Zone 7: Strafe Left
                frontLeftDrive.setPower(-leftStickX);
                frontRightDrive.setPower(leftStickX);
                backRightDrive.setPower(-leftStickX);
                backLeftDrive.setPower(leftStickX);
            }  else if (leftStickX < 0 && leftStick > .4 && leftStick < .8) { //Zone 8: Diagonal FL
                backLeftDrive.setPower(leftStickX);
                frontRightDrive.setPower(leftStickX);
            }

            // turning
            frontLeftDrive.setPower(rightStickX);
            backLeftDrive.setPower(rightStickX);
            frontRightDrive.setPower(-rightStickX);
            backRightDrive.setPower(-rightStickX);
        }


        // Actuator control - A up; B down
        if (gamepad1.a) {
            actuator.setPower(1);
        } else if (gamepad1.b) {
            actuator.setPower(-1);
        } else {
            actuator.setPower(0);
        }

        // Reving - joke (like the cars in movies)
        if (gamepad1.x) {
            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
            backLeftDrive.setPower(1);
            backRightDrive.setPower(1);
            etime.reset();
            while (etime.time() < .1) ;

            frontLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
            backLeftDrive.setPower(-1);
            backRightDrive.setPower(-1);
            etime.reset();
            while (etime.time() < .1) ;

            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }

        // control arm's shoulder and elbow with joysticks
        shoulder.setPower(-gamepad2.left_stick_y);
        elbow.setPower(-gamepad2.right_stick_y / 2.5);

        // control servo with A & B
        if (gamepad2.b) {
            intake.setPosition(intake.getPosition() - .01);
        } else if (gamepad2.a) {
            intake.setPosition(intake.getPosition() + .01);
        }

        // slight robot movements on the subsystems gamepad
        if (gamepad2.dpad_up) {
            frontLeftDrive.setPower(.4);
            frontRightDrive.setPower(.4);
            backLeftDrive.setPower(.4);
            backRightDrive.setPower(.4);
        } else if (gamepad2.dpad_down) {
            frontLeftDrive.setPower(-.4);
            frontRightDrive.setPower(-.4);
            backLeftDrive.setPower(-.4);
            backRightDrive.setPower(-.4);
        } else if (gamepad2.dpad_left) {
            frontLeftDrive.setPower(-.4);
            frontRightDrive.setPower(.4);
            backLeftDrive.setPower(-.4);
            backRightDrive.setPower(.4);
        } else if (gamepad2.dpad_right) {
            frontLeftDrive.setPower(.4);
            frontRightDrive.setPower(-.4);
            backLeftDrive.setPower(.4);
            backRightDrive.setPower(-.4);
        }
    }
}
