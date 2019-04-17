package org.firstinspires.ftc.teamcode.FinalCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class AutoImuRange extends LinearOpMode {

    // Drive system
    private DcMotor FL, FR, BL, BR;

    // IMU
    BNO055IMU imu;
    Orientation angles;

    // Ultra sonic
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        mapDriveSystems();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        waitForStart();

        //moveUntil(2.0);
        //turnImu(90.0);
    }

    private void stopDrive() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void moveUntil(double target) {
        stopDrive();
        double curDist = rangeSensor.getDistance(DistanceUnit.CM);

        while (opModeIsActive() && !(curDist > target - 0.2 && curDist < target + 0.2)) {
            curDist = rangeSensor.getDistance(DistanceUnit.CM);
            if (curDist > target) {
                FL.setPower(.6);
                FR.setPower(.6);
                BL.setPower(.6);
                BR.setPower(.6);
            } else if (curDist < target) {
                BR.setPower(-.2);
                FL.setPower(-.2);
                FR.setPower(-.2);
                BL.setPower(-.2);
            }
        }
        stopDrive();
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    private void turnImu(float degrees) {
        stopDrive();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float targetAngle = angles.thirdAngle - degrees;
        while (opModeIsActive() && !(angles.thirdAngle < targetAngle + 1.0 && angles.thirdAngle > targetAngle + 1.0)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (targetAngle < angles.thirdAngle) {
                FL.setPower(-.6);
                FR.setPower(-.6);
                BL.setPower(-.6);
                BR.setPower(-.6);
            } else if (targetAngle > angles.thirdAngle) {
                FL.setPower(.6);
                FR.setPower(.6);
                BL.setPower(.6);
                BR.setPower(.6);
            } else {
                break;      // should never reach here
            }
        }
        stopDrive();
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
}
