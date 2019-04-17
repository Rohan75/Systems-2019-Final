package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class Methods extends LinearOpMode {

    // Drive System
    DcMotor FL, FR, BL, BR;

    // Sub Systems
    DcMotor actuator, shoulder, elbow;
    Servo intake;

    // Encoder values
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 24, GEAR_RATIO = 60,
    TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = (TICKS_PER_ROT * GEAR_RATIO) / (2 * Math.PI * WHEEL_RADIUS),
    TURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));
    private static final double AndyTICKS_PER_REV = 1120, AndyTICKS_PER_DEGREE = AndyTICKS_PER_REV / 360,
    AndyTICKS_PER_INCH = AndyTICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS),
    AndyTURN_RADIUS = Math.sqrt(Math.pow((double) LENGTH / 2, 2) + Math.pow((double) WIDTH / 2, 2));

    // VuForia Stuff
    private VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    ElapsedTime etime = new ElapsedTime();

    @Override
    abstract public void runOpMode() throws InterruptedException;

    abstract void mapDriveSystem();

    abstract void mapSubSystems();

    // standard drive; enter inch/geree measurement, and the units (use negative for moving backward)
    void moveZ(String type, double value) {
        type = type.toLowerCase();
        if (type.equals("inches")) {
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * AndyTICKS_PER_INCH)));
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
        while (opModeIsActive() && (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) ;
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // strafing; enter inch/degree measurement, and the units (use negative for strafing left)
    void moveX(String type, double value) {
        type = type.toLowerCase();
        if (type.equals("inches")) {
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * AndyTICKS_PER_INCH)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (value * TICKS_PER_INCH)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (value * TICKS_PER_INCH)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_INCH)));
        } else if (type.equals("degrees")) {
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (value * AndyTICKS_PER_DEGREE)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (value * TICKS_PER_DEGREE)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (value * TICKS_PER_DEGREE)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
        }

        FL.setPower(.6);
        FR.setPower(.6);
        BL.setPower(.6);
        BR.setPower(.6);
        while (opModeIsActive() && (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) ;
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // turning; positive degrees for right, negative degrees for left
    void turnY(double degrees) {
        double turnInches = (degrees / 360) * (2 * Math.PI * TURN_RADIUS);
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (turnInches * AndyTICKS_PER_INCH)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (turnInches * TICKS_PER_INCH)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (turnInches * TICKS_PER_INCH)));

        FL.setPower(.6);
        FR.setPower(.6);
        BL.setPower(.6);
        BR.setPower(.6);
        while (opModeIsActive() && (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) ;
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    /**
     * Use for mineral Detection
     *
     * Copy & Paste this code when facing left mineral

         List<Recognition> _objects = null;
         while (_objects == null) {
         _objects = tfod.getUpdatedRecognitions();
         }
         sampling(_objects, 5);

     */
    void sampling(List<Recognition> objectsFound, int timer) {
        initVuforia();
        initTfod();
        etime.reset();

        if (objectsFound != null) {
            if (objectsFound.size() == 1) {
                while (opModeIsActive() && objectsFound.get(0).getLabel().equals("Gold Mineral") &&
                        etime.time() < timer) {
                    objectsFound = null;
                    while (opModeIsActive() && objectsFound == null && etime.time() < timer) {
                        turnY(10);
                        objectsFound = tfod.getUpdatedRecognitions();
                    }
                }
                moveZ("inches", 15);
                return;
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AU4FDNH/////AAABmRVll+1KLUOvurFViKcVqNpTWXwbjf31OgDqmYXYdxSUgDANPXZ2u318WCLJ72KycjRKeDz92M2BmL8lNtnE5seRlMt7ES28yoMbkp1ic0xgmLAo1f1tXFBySj9WFJD708PscrGLeGr8vbbhDb6Zmv1t4i+ZEsHZVQyijBGH6t+egvvwjYdqA+tOZdujYxX1TWmGXJQVwI1PpA9YFh8+m2DfFteZ7zyyirYeFllW03a8yHZZyeVF0GycAPn9nBkPEDWEFodD9S2/mKJLdq/FyPIyfbh/9v6K8biIL0UyyOzIKEI3N542XZspTv6RkWeKcMwqOZqaPXBtDWnX8uxJ3gaq0Zbb2t12cBBBjVj5d5a/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
        tfod.activate();
    }

}








