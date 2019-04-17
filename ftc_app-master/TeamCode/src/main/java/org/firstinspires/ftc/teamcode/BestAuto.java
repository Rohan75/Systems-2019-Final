package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class BestAuto extends LinearOpMode {

    // Drive System
    private DcMotor FL, FR, BL, BR;

    // Sub-systems
    DcMotor actuator, shoulder, elbow;
    Servo intake, marker;

    // Encoder values
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 24, GEAR_RATIO = 60;
    private static final double TICKS_PER_INCH = (TICKS_PER_ROT * GEAR_RATIO) / (2 * Math.PI * WHEEL_RADIUS),
            TURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));
    private static final double AndyTICKS_PER_REV = 1120,
            AndyTICKS_PER_INCH = AndyTICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS);

    // Sampling object detection
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        tfod.activate();
        initVuforia();
        initTfod();

        mapDrive();
        mapSubSystems();

        waitForStart();

        // Sampling
        tfod.activate();
        drive('t', -30);
        sample(30);

        // Claiming
        drive('x', -25);
        drive('t', 45);
        drive('y', 30);

        // Parking
        drive('y', 70);
    }

    void mapDrive() {
        // Map variables to phone
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse one side
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop immediately
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run using encoders
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void mapSubSystems() {
        // Map lift
        actuator = hardwareMap.get(DcMotor.class, "actuator");

        // Map intake system
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(Servo.class, "intake");

        // Stop immediately
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map team marker
        marker = hardwareMap.get(Servo.class, "marker");
    }

    /**
     *
     * @param axis - x (strafe), y (forwards/backward), or t (turn)
     * @param value - the distance to move in inches
     *              For y: positive is forward & negative is backward
     *              For x: positive is right & negative is left
     *              For t: positive is counterclockwise & negative is clockwise
     * @param speed - the speed to move at; default it .5
     */
    void drive(char axis, double value, double... speed) {
        double _speed = (speed.length > 0 ? speed[0] : .5);

        value = (axis == 't' ? value/360 *  (2 * Math.PI * TURN_RADIUS) : value);

        int fl = FL.getCurrentPosition() + (axis == 'y' ? 1 : -1) * (int)(value * AndyTICKS_PER_INCH);
        int fr = FL.getCurrentPosition() + (int)(value * TICKS_PER_INCH);
        int bl = FL.getCurrentPosition() + (axis == 't' ? -1 : 1) * (int)(value * TICKS_PER_INCH);
        int br = FL.getCurrentPosition() + (axis == 'x' ? -1 : 1) * (int)(value * TICKS_PER_INCH);

        FL.setTargetPosition(fl);
        FR.setTargetPosition(fr);
        BL.setTargetPosition(bl);
        BR.setTargetPosition(br);

        FL.setPower(_speed);
        FR.setPower(_speed);
        BL.setPower(_speed);
        BR.setPower(_speed);
        while ((FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()) && opModeIsActive());
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    void sample(int degreesRight) {
        boolean done = false;
        int degreesTurned = 0;
        while (!done) {
            List<Recognition> minerals = tfod.getUpdatedRecognitions();
            if (minerals.size() == 1) {
                if (minerals.get(1).getLabel().equals("Gold Mineral")) {
                    drive('y', 15);
                    done = true;
                } else {
                    drive('t', 10);
                    degreesTurned += 10;
                }
            }
        }
        drive('y', -15);
        drive('t', -(degreesTurned - degreesRight));
    }

    void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AU4FDNH/////AAABmRVll+1KLUOvurFViKcVqNpTWXwbjf31OgDqmYXYdxSUgDANPXZ2u318WCLJ72KycjRKeDz92M2BmL8lNtnE5seRlMt7ES28yoMbkp1ic0xgmLAo1f1tXFBySj9WFJD708PscrGLeGr8vbbhDb6Zmv1t4i+ZEsHZVQyijBGH6t+egvvwjYdqA+tOZdujYxX1TWmGXJQVwI1PpA9YFh8+m2DfFteZ7zyyirYeFllW03a8yHZZyeVF0GycAPn9nBkPEDWEFodD9S2/mKJLdq/FyPIyfbh/9v6K8biIL0UyyOzIKEI3N542XZspTv6RkWeKcMwqOZqaPXBtDWnX8uxJ3gaq0Zbb2t12cBBBjVj5d5a/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
