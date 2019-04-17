/*
    Syborgs 10696 Autonomous
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")
public class Autonomous extends LinearOpMode {

    // Drive system
    private static DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Sub-systems
    private static DcMotor actuator;
    private static DcMotor shoulder, elbow;
    private static Servo intake, marker;

    // Encoder equations (torquenado) - See notebook pages E-5 & E-6
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 24, GEAR_RATIO = 60,
            TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = (TICKS_PER_ROT * GEAR_RATIO) / (2 * Math.PI * WHEEL_RADIUS),
            TURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));
    // Neverest motors have different ticks
    private static final double AndyTICKS_PER_REV = 1120, AndyTICKS_PER_DEGREE = AndyTICKS_PER_REV / 360,
            AndyTICKS_PER_INCH = AndyTICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS),
            AndyTURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));

    ElapsedTime etime = new ElapsedTime();

    // INIT
    @Override
    public void runOpMode() throws InterruptedException {

        // hardwareMap devices used
        mapDriveSystem();
        mapSubSystems();


        // START
        waitForStart();

        // Raise arm's shoulder to get it out of the way
        shoulder.setPower(1);
        sleep(500);
        shoulder.setPower(0);

        // Raise actuator to lower robot
        actuator.setPower(.8);
        sleep(6750);
        actuator.setPower(0);

        // Get off lander
        Drive.moveZ("inches", -.6);
        nothing();
        Drive.moveX("inches", -7, .3);
        nothing();

        // Claiming
        Drive.moveZ("inches", -60);
        nothing();
        Drive.turnY(-80);
        nothing();
        marker.setPosition(.4);
        Drive.moveX("inches", 3);
        nothing();

        // Parking
        Drive.turnY(-108);
        nothing();
        Drive.moveZ("inches", -70);
        nothing();
        shoulder.setPower(1);
        sleep(500);
        shoulder.setPower(0);
        elbow.setPower(-1);
        sleep(1500);
        elbow.setPower(0);
    }

    /**
     * maps the drive motors to the variables
     */
    void mapDriveSystem() {
        // map motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // reverse left side
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // stop immediately
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset encoders to prevent gigantic numbers
        frontLeftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        // set motors to encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    /**
     * Stop the program while the wheels move to the encoder position
     * Called directly after the Drive class methods
     */
    void nothing() {
        // stop program while motors haven't reached encoder target
        while (opModeIsActive() && backRightDrive.isBusy());

        // turn off motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Instantiating subsystem motors/servos
     */
    void mapSubSystems() {
        // map subsystems
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(Servo.class, "intake");
        actuator = hardwareMap.dcMotor.get("actuator");
        marker = hardwareMap.get(Servo.class, "marker");
    }

    /**
     * class containing all the methods for driving
     */
    public static class Drive {

        /**
         * standard drive; enter inch/degree measurement, and the units (use negative for moving backward)
         */
        static void moveZ(String inches_or_degrees, double value, Double... _speed) {
            inches_or_degrees = inches_or_degrees.toLowerCase();

            // inches or degrees
            if (inches_or_degrees.equals("inches")) {
                // set encoder target positions based on entered inches

                frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + (value * AndyTICKS_PER_INCH)));

                frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() + (value * TICKS_PER_INCH)));

                backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() + (value * TICKS_PER_INCH)));

                backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() + (value * TICKS_PER_INCH)));

            } else if (inches_or_degrees.equals("degrees")) {
                // set encoder target positions based on entered inches degrees

                frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + (value * AndyTICKS_PER_DEGREE)));

                frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

                backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

                backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
            }

            // turn on motors; let them move to position
            double speed = (_speed.length > 0 ? _speed[0] : .6);
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(speed);
            backRightDrive.setPower(speed);
        }

        // strafing; enter inch/degree measurement, and the units (use negative for strafing left)
        static void moveX(String inches_or_degrees, double value, Double... _speed) {
            inches_or_degrees = inches_or_degrees.toLowerCase();
            if (inches_or_degrees.equals("inches")) {
                // set encoder target positions based on entered inches

                frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + (value * AndyTICKS_PER_INCH)));

                frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() - (value * TICKS_PER_INCH)));

                backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() - (value * TICKS_PER_INCH)));

                backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() + (value * TICKS_PER_INCH)));

            } else if (inches_or_degrees.equals("degrees")) {
                // set encoder target positions based on entered degrees

                frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + (value * AndyTICKS_PER_DEGREE)));

                frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() - (value * TICKS_PER_DEGREE)));

                backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() - (value * TICKS_PER_DEGREE)));

                backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

            }

            double speed = (_speed.length > 0 ? _speed[0] : .6);
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(speed);
            backRightDrive.setPower(speed);
        }

        // turning; positive degrees for right, neagtive degrees for left
        static void turnY(double degrees, Double... _speed) {
            // set encoder position using radius of robot
            double turnInches = (degrees / 360) * (2 * Math.PI * TURN_RADIUS);

            frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + (turnInches * AndyTICKS_PER_INCH)));

            frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));

            backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() + (turnInches * TICKS_PER_INCH)));

            backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));

            double speed = (_speed.length > 0 ? _speed[0] : .6);
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(speed);
            backRightDrive.setPower(speed);
        }
    }

}






