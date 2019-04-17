package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous
public class LastYearCode extends LinearOpMode
{
    // Declaration of Motors.
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor Lift;
    DcMotor Spin;

    // Declaration of Servos.
    Servo TL;
    Servo BML;
    Servo TBR;
    Servo MR;
    Servo Tilt;
    Servo Pan;

    // Declarating and Assigning Timing Variable.
    ElapsedTime etime = new ElapsedTime();

    // Declaration of Vuforia Variable.
    VuforiaLocalizer vuforia;


    // Declaration of Colro Sensor.
    ColorSensor sensorColor;

    // Declaration of Gyro Sensor Variables.
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;
    int zAccumulated;

    public void runOpMode()
    {
        // Assign Servos and Motors to Corresponding Hardwaremap name.
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        Lift = hardwareMap.dcMotor.get("L");
        Spin = hardwareMap.dcMotor.get("Sp");
        TL = hardwareMap.servo.get("TL");
        BML = hardwareMap.servo.get("BML");
        TBR = hardwareMap.servo.get("TBR");
        MR = hardwareMap.servo.get("MR");
        Tilt = hardwareMap.servo.get("JA");
        Pan = hardwareMap.servo.get("JS");

        // Set the Pan and Tilt Servo to Their Initial Positions.
        Pan.setPosition(.43);
        Tilt.setPosition(1);

        // Set Motor Zero Power Behavior to BRAKE, allowing for more precision.
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Vuforia Initialization.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AfWRBkH/////AAAAmek/nVPS1EFvvAyu/zJh91BoRaperi48rw3ELqFsIfWYonN7TX8Kli7CE4mMwC0THatOzal+EfQGLHRJlwknLVhSu9s36yp4w7/ZlSRnYdBzIrHvCMYFxhWfX64BhZ9t1hRtSlZMQvTv9kaZ4ZeZp/9kYbOl2PAjLhwLyi787IRVyDkENRDXzcrArgCVw5GFMWXdaV7GAwZ0DDAm17BKnvmfSC/te0XQeVA+GpIIOmvfCISy1Ke/ngMqmoH3FrJyKt3HSwgMxivkDwxDmD39I0hYoMywsZRvSf+sIXgIxUjKT0ePTtuSbUz2mLxf0gllKTK4i1My/ubxrscmWjeexcoB+Md58htvCMLK6qit7xQk";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        // Color Sensor Initialization.
        sensorColor = hardwareMap.get(ColorSensor.class, "cs");
        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        // Gyro Sensor Initialization.
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
        mrGyro.calibrate();

        // Wait Until the Start Button is Pressed.
        waitForStart();

        // Wait Until the Gyrosensor Has Completed Calibration.
        while (mrGyro.isCalibrating())
        {
            telemetry.addData("LOADING", "");
        }

        // Begin Tracking for Pictographs.
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        // Until a Pictograph is Identified, Wait.
        while (vuMark == RelicRecoveryVuMark.UNKNOWN)
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        // Tilt Color Sensor Between the Jewels.
        Tilt.setPosition(.42);
        pause(1.5);

        // Convert the RGB Values to HSV Values, Multiply by the SCALE_FACTOR,
        // Then Cast it Back to an Integer.
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // If More Red is Detected Then Blue, Then Knock off The Left Jewel.
        if (sensorColor.red() > sensorColor.blue())
        {
            Pan.setPosition(.1);
        }
        // If More Blue is Detected Then Red, Then Knock off The Right Jewel.
        else if (sensorColor.blue() > sensorColor.red())
        {
            Pan.setPosition(.8);
        }
        pause(1);

        // Reset the Pan and Tilt to Be Inside the Robot.
        Tilt.setPosition(.95);
        pause(1);
        Pan.setPosition(.52);
        pause(1);

        // Power The Intake Servos; This is To Ensure The Glyph Remains in
        // The Robot Until We Align With The Cryptobox.
        TL.setPosition(0.1);
        BML.setPosition(0.1);
        TBR.setPosition(.9);
        MR.setPosition(0.1);

        // Slowly Accelerate Off of The Balancing Board.
        backward(.2, .1);
        backward(.2, .2);
        backward(1.45, .3);
        pause(.5);

        // Turn Until We Are Facing the Cryptobox, and Parallel With The Wall.
        zAccumulated = mrGyro.getIntegratedZValue();
        while (zAccumulated < 162)
        {
            FL.setPower(-.4);
            FR.setPower(.4);
            BL.setPower(-.4);
            BR.setPower(.4);
            zAccumulated = mrGyro.getIntegratedZValue();
        }
        pause(.5);

        // Strafe Rightwards For A Certain Time Depending On Which Pictograph
        // Was Detected.
        if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            rShift(1.73, .4);
        } else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            rShift(.87, .4);
        } else
        {
            rShift(.71, .4);
        }
        pause(.5);

        // Drive Forward Slightly.
        forward(.21, .2);
        pause(.5);

        // Dispense In Front Of The Cipher, But Not Into It.
        dispense();
        pause(.5);

        // Drive Forward and Dispense Simultaneously, Pushing The Block Into The Cipher.
        forward(.8, .2);
        TL.setPosition(1);
        BML.setPosition(1);
        TBR.setPosition(0);
        MR.setPosition(1);
        pause(.5);

        // Drive Backwards and Stop Dispensing.
        backward(.4, .4);
        TL.setPosition(0.5);
        BML.setPosition(0.5);
        MR.setPosition(0.4826);
        TBR.setPosition(0.5);
    }

    // Drives the Robot Forward for a Specific Amount of Time.
    public void forward(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(-spd);
            FR.setPower(-spd);
            BL.setPower(-spd);
            BR.setPower(-spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    // Drives the Robot Backward for a Specific Amount of Time.
    public void backward(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(spd);
            FR.setPower(spd);
            BL.setPower(spd);
            BR.setPower(spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    // Shifts the Robot to the Left for a Specific Amount of Time.
    public void lShift(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(-spd);
            FR.setPower(spd);
            BL.setPower(spd);
            BR.setPower(-spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Shifts the Robot to the Right for a Specific Amount of Time.
    public void rShift(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(spd);
            FR.setPower(-spd);
            BL.setPower(-spd);
            BR.setPower(spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    // Rotates the Robot to the Left for a Specific Amount of Time.
    public void lRotate(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(-spd);
            FR.setPower(spd);
            BL.setPower(-spd);
            BR.setPower(spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    // Rotates the Robot to the Right for a Specific Amount of Time.
    public void rRotate(double timeVal, double spd)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(spd);
            FR.setPower(-spd);
            BL.setPower(spd);
            BR.setPower(-spd);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    // Dispenses the Glyph.
    public void dispense()
    {
        etime.reset();
        while (etime.time() < 1.3)
        {
            TL.setPosition(1);
            BML.setPosition(1);
            TBR.setPosition(0);
            MR.setPosition(1);
        }
        TL.setPosition(0.5);
        BML.setPosition(0.5);
        MR.setPosition(0.4826);
        TBR.setPosition(0.5);
    }


    // Pauses the Robot's Movements For a Specific Amount of Time.
    public void pause(double timeVal)
    {
        etime.reset();
        while (etime.time() < timeVal)
        {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
}
