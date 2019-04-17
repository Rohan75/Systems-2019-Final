
/**
 * Team:    Syosset Syborgs
 * ID:      10696
 *
 * Final Depot Side Autonomous
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RohanAutoDepot", group = "Autonomous")
public class FinalAutoDepot extends LinearOpMode {

    // Drive system
    private static DcMotor FL, FR, BL, BR;

    // Sub-systems
    private static DcMotor actuator;
    private static DcMotor shoulder, elbow;
    private static Servo marker;
    static CRServo intake;

    // Encoder values
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 24, GEAR_RATIO = 60,
            TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = (TICKS_PER_ROT * GEAR_RATIO) / (2 * Math.PI * WHEEL_RADIUS),
            TURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));
    private static final double AndyTICKS_PER_REV = 1120, AndyTICKS_PER_DEGREE = AndyTICKS_PER_REV / 360,
            AndyTICKS_PER_INCH = AndyTICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS),
            AndyTURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));

    // VuForia + TensorFlow declarations
    private static final String VUFORIA_KEY = "AU4FDNH/////AAABmRVll+1KLUOvurFViKcVqNpTWXwbjf31OgDqmYXYdxSUgDANPXZ2u318WCLJ72KycjRKeDz92M2BmL8lNtnE5seRlMt7ES28yoMbkp1ic0xgmLAo1f1tXFBySj9WFJD708PscrGLeGr8vbbhDb6Zmv1t4i+ZEsHZVQyijBGH6t+egvvwjYdqA+tOZdujYxX1TWmGXJQVwI1PpA9YFh8+m2DfFteZ7zyyirYeFllW03a8yHZZyeVF0GycAPn9nBkPEDWEFodD9S2/mKJLdq/FyPIyfbh/9v6K8biIL0UyyOzIKEI3N542XZspTv6RkWeKcMwqOZqaPXBtDWnX8uxJ3gaq0Zbb2t12cBBBjVj5d5a/";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        mapDriveSystem();
        mapSubSystems();

        initVuforia();
        initTfod();

        waitForStart();

        // Stabilize team marker
        marker.setPosition(1);

        // Raise actuator until robot is on ground
        actuator.setPower(1);
        sleep(5910);
        actuator.setPower(-1);
        sleep(180);
        actuator.setPower(0);

        // Turn off encoders
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Back up to get front wheels on ground
        BL.setPower(-.2);
        BR.setPower(-.2);
        FL.setPower(-.2);
        FR.setPower(-.2);
        sleep(400);

        // Strafe right, off lander
        BL.setPower(-.4);
        BR.setPower(.4);
        FL.setPower(.4);
        FR.setPower(-.4);
        sleep(870);
        BL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);

        // Strafe left, away from hook
        BL.setPower(.4);
        BR.setPower(-.4);
        FL.setPower(-.4);
        FR.setPower(.4);
        sleep(400);
        BL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);

        // turn on encoders
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Line up with left most mineral
        Drive.moveZ("inches", -20);
        nothing();
        Drive.moveX("inches", 11, .6);
        nothing();

        // Sampling code
        sample();
        // Move to depot, but at a different angle if mineral is on the left
        if (sampTime <= .5) {
            Drive.turnY(-45);
            nothing();
            Drive.moveX("inches", -6, .6);
            nothing();
        } else {
            Drive.turnY(-90);
            nothing();
        }

        // Drop team marker
        marker.setPosition(.2);
        Drive.moveX("inches", 8, .8);
        nothing();

        // Move out of depot for alliance
        Drive.moveZ("inches", -5);
        nothing();

    }

    // Instantiating motors for four wheel drive; mecanum wheels
    void mapDriveSystem() {
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotor.RunMode.RESET_ENCODERS);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    // Call this method after a drive method
    void nothing() {
        while (opModeIsActive() && BR.isBusy());
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Instantiating subsystem motors/servos
    void mapSubSystems() {
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(CRServo.class, "intake");
        actuator = hardwareMap.dcMotor.get("actuator");
        marker = hardwareMap.get(Servo.class, "marker");
    }

    // Class with methods used in drive system
    public static class Drive {

        // Standard drive; enter inch/degree measurement, and the units (use negative for moving backward)
        static void moveZ(String type, double value) {
            type = type.toLowerCase();
            if (type.equals("inches")) {
                FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * TICKS_PER_INCH)));

                FR.setTargetPosition((int) (FR.getCurrentPosition() + (value * TICKS_PER_INCH)));

                BL.setTargetPosition((int) (BL.getCurrentPosition() + (value * TICKS_PER_INCH)));

                BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_INCH)));

            } else if (type.equals("degrees")) {
                FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * AndyTICKS_PER_DEGREE)));

                FR.setTargetPosition((int) (FR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

                BL.setTargetPosition((int) (BL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

                BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
            }
            FL.setPower(.6);
            FR.setPower(.6);
            BL.setPower(.6);
            BR.setPower(.6);
        }

        // Strafing; enter inch/degree measurement, and the units (use negative for strafing left)
        static void moveX(String type, double value, double speed) {
            type = type.toLowerCase();
            if (type.equals("inches")) {
                FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * TICKS_PER_INCH)));

                FR.setTargetPosition((int) (FR.getCurrentPosition() - (value * TICKS_PER_INCH)));

                BL.setTargetPosition((int) (BL.getCurrentPosition() - (value * TICKS_PER_INCH)));

                BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_INCH)));

            } else if (type.equals("degrees")) {
                FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * AndyTICKS_PER_DEGREE)));

                FR.setTargetPosition((int) (FR.getCurrentPosition() - (value * TICKS_PER_DEGREE)));

                BL.setTargetPosition((int) (BL.getCurrentPosition() - (value * TICKS_PER_DEGREE)));

                BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));

            }
            FL.setPower(speed);
            FR.setPower(speed);
            BL.setPower(speed);
            BR.setPower(speed);
        }

        // Turning; positive degrees for right, negative degrees for left
        static void turnY(double degrees) {
            double turnInches = (degrees / 360) * (2 * Math.PI * TURN_RADIUS);
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (turnInches * TICKS_PER_INCH)));

            FR.setTargetPosition((int) (FR.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));

            BL.setTargetPosition((int) (BL.getCurrentPosition() + (turnInches * TICKS_PER_INCH)));

            BR.setTargetPosition((int) (BR.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));
            FL.setPower(.6);
            FR.setPower(.6);
            BL.setPower(.6);
            BR.setPower(.6);

        }
    }

    private double sampTime;
    private void sample() {
        // Stop watch for identifying which mineral position was pushed
        ElapsedTime etime = new ElapsedTime();

        // Turn off encoders
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turn to adjust angle of robot relative to minerals
        FL.setPower(.5);
        BL.setPower(.5);
        sleep(200);
        FL.setPower(0);
        BL.setPower(0);

        // Turn on TensorFlow
        tfod.activate();

        // Booleans for while in sampling
        boolean done = false;
        boolean timedOut = false;

        // Start/reset stopwatch
        etime.reset();
        while (!done && opModeIsActive() && !timedOut) {
            if (etime.time() >= 5.5) {  // If robot spent more than 5.5 seconds sampling, stop
                timedOut = true;
            }
            List<Recognition> minerals = tfod.getUpdatedRecognitions(); // Get list of identified minerals
            if (minerals != null) {
                if (minerals.size() == 1) {     // Check if a mineral is detected
                                                // Turned off silver mineral detection
                    // Stop strafing (see line 323)
                    BL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);

                    // Turn on encoders
                    FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Strafe left - phone is positioned on left; must move left to center mineral
                    Drive.moveX("inches", 13, .6);
                    nothing();
                    // Move forward; move less if mineral is in left position
                    if (sampTime <= .5) {
                        Drive.moveZ("inches", -32);
                    } else {
                        Drive.moveZ("inches", -40);
                    }
                    nothing();
                    done = true; // Exit loop
                }
            }
            else {  // Strafe right if no mineral detected
                BL.setPower(.35);
                BR.setPower(-.35);
                FL.setPower(-.35);
                FR.setPower(.35);
            }
        }
        if (timedOut) { // Move forward more if mineral is in right position
            BL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Drive.moveZ("inches", -48);
            nothing();
        }
        tfod.shutdown();    // Turn off TensorFlow

    }

    void initVuforia() {
        // Vuforia paramaters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY; // License key
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Turn on camera

        vuforia = ClassFactory.getInstance().createVuforia(parameters); // Turn on Vuforia
    }

    void initTfod() {
        // Initialize TensorFlow to see only gold minerals
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral");
    }

}






