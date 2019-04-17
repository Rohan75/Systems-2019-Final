/*
 * Team:    Syosset Syborgs
 * ID:      10696
 *
 * Author:  Rohan Ghotra        GitHub User: Rohan75
 * Version: v2.1
 */

// NOTE: This code is currently disabled and unresponsive, do not attempt to use

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class NewAutonomous extends LinearOpMode {

    // Drive system
    private static DcMotor FL, FR, BL, BR;

    // Sub-systems
    private static DcMotor actuator;
    private static Servo shoulder, elbow, wrist;
    private static CRServo intake;
    private static boolean shouldIntake = false;

    // Encoder values
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 1440, GEAR_RATIO = 60,
            TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = TICKS_PER_ROT * GEAR_RATIO / 2 * Math.PI * WHEEL_RADIUS,
            TURN_RADIUS = Math.sqrt(Math.pow((double)LENGTH/2, 2) + Math.pow((double)WIDTH/2, 2));

    // VuForia + TensorFlow declarations
    private static final String VUFORIA_KEY = "AU4FDNH/////AAABmRVll+1KLUOvurFViKcVqNpTWXwbjf31OgDqmYXYdxSUgDANPXZ2u318WCLJ72KycjRKeDz92M2BmL8lNtnE5seRlMt7ES28yoMbkp1ic0xgmLAo1f1tXFBySj9WFJD708PscrGLeGr8vbbhDb6Zmv1t4i+ZEsHZVQyijBGH6t+egvvwjYdqA+tOZdujYxX1TWmGXJQVwI1PpA9YFh8+m2DfFteZ7zyyirYeFllW03a8yHZZyeVF0GycAPn9nBkPEDWEFodD9S2/mKJLdq/FyPIyfbh/9v6K8biIL0UyyOzIKEI3N542XZspTv6RkWeKcMwqOZqaPXBtDWnX8uxJ3gaq0Zbb2t12cBBBjVj5d5a/";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ArrayList<minerals> sampling;

    // Timer
    private static ElapsedTime etime = new ElapsedTime();

    public enum minerals {
        GOLD, SILVER
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        mapDriveSystem();

        waitForStart();


        tfod.activate();
        while (opModeIsActive() && sampling == null) {
            sampling = objectDetection(tfod.getUpdatedRecognitions());
        }
        telemetry.addData("MotorPosition", FL.getCurrentPosition());
        telemetry.update();
        if (sampling.indexOf(minerals.GOLD) == 0) {
            drive('x', -16);
            drive('y', 10);
        } else if (sampling.indexOf(minerals.GOLD) == 1) {
            drive('y', 10);
        } else {
            drive('x', 16);
            drive('y', 10);
        }
    }

    /**
     *
     * @param axis - x (strafe, y (forwards/backward), or t (turn)
     * @param value - the distance to move in inches
     *              For y: positive is forward & negative is backward
     *              For x: positive is right & negative is left
     *              For t: positive is counterclockwise & negative is clockwise
     * @param speed - the speed to move at; default it .5
     */
    void drive(char axis, double value, double... speed) {
        double _speed = (speed.length > 0 ? speed[0] : .5);

        value = (axis == 't' ? value/360 *  (2 * Math.PI * TURN_RADIUS) : value);

        int fl = FL.getCurrentPosition() + (axis == 'y' ? 1 : -1) * (int)(value * TICKS_PER_INCH);
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
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy() && opModeIsActive());
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Instantiating motors for four wheel drive; mecanum wheels
    public void mapDriveSystem() {
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

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public ArrayList<minerals> objectDetection(List<Recognition> objectsFound) {
        ArrayList<minerals> order = new ArrayList<>();

        if (objectsFound.size() == 3) {
            int goldPos = 0;
            int silverPos1 = 0;
            int silverPos2 = 0;

            for (Recognition object : objectsFound) {
                if (object.getLabel().equals("Gold Mineral")) {
                    goldPos = (int) object.getLeft();
                } else if (silverPos1 == 0) {
                    silverPos1 = (int) object.getLeft();
                } else {
                    silverPos2 = (int) object.getLeft();
                }
            }

            if (goldPos < silverPos1 && goldPos < silverPos2) {
                order.add(0, minerals.GOLD);
                order.add(1, minerals.SILVER);
                order.add(2, minerals.SILVER);
            } else if (silverPos1 < goldPos && goldPos < silverPos2) {
                order.add(0, minerals.SILVER);
                order.add(1, minerals.GOLD);
                order.add(2, minerals.SILVER);
            } else if (silverPos1 < goldPos && silverPos2 < goldPos) {
                order.add(0, minerals.SILVER);
                order.add(1, minerals.SILVER);
                order.add(2, minerals.GOLD);
            } else {
                return null;
            }
        }

        if (order.size() == 3) {
            return order;
        } else {
            return null;
        }
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
    }

}
