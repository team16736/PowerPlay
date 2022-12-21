package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.constants.MotorConstants;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;


//moves forward to the carousel, spins it, then turns and parks in the storage unit

@TeleOp(name = "KindOfBasicDrive")
public class BasicDrive extends LinearOpMode {

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;

    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

//    public ColorSensor coneSensor;
    public DistanceSensor junctionSensor;

    public double THROTTLE = 0.25;

    public double red = 0;
    public double green = 0;
    public double blue = 0;

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

//        coneSensor = hardwareMap.get(ColorSensor.class, "coneSensor");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");

        double prevTime = System.currentTimeMillis();
        double currentTime = 0;


        setMotorDirection_Forward();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        double startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            drive(
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)),      //joystick controlling strafe
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)),     //joystick controlling forward/backward
                    (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)));    //joystick controlling rotation
//            telemetry.addData("Seeing Color", mostColor());
            currentTime = System.currentTimeMillis();
            telemetry.addData("Interval", currentTime - prevTime);
            prevTime = currentTime;
            telemetry.addData("Distance", junctionSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    public void drive(double speedX, double speedY, double rotation) {

        double throttledX = speedX * THROTTLE;
        double throttledY = speedY * THROTTLE;
        double throttledRotation = rotation * THROTTLE;

        driveUsingJoyStick(throttledX, throttledY, throttledRotation);
    }

    public void driveUsingJoyStick(double speedX, double speedY, double rotation) {

        double frontLeft = speedX + speedY + rotation;
        double frontRight = -speedX + speedY - rotation;

        double backLeft = -speedX + speedY + rotation;
        double backRight = speedX + speedY - rotation;

//        double fl = speedX + speedY + rotation;
//        double fr = -speedX + speedY - rotation;
//        double bl= -speedX + speedY + rotation;
//        double br = speedX + speedY - rotation;

        double max = getMaxPower(frontLeft, frontRight, backLeft, backRight);
        if (max > 1) {
            frontLeft = frontLeft / max;
            frontRight = frontRight / max;
            backLeft = backLeft / max;
            backRight = backRight / max;
        }

        rightFront.setPower(frontRight);
        leftFront.setPower(frontLeft);
        rightRear.setPower(backRight);
        leftRear.setPower(backLeft);
    }

    private double getMaxPower(double frontLeftValue, double frontRightValue, double backLeftValue, double backRightValue) {
        List<Double> valueList = new LinkedList<>();
        valueList.add(frontLeftValue);
        valueList.add(frontRightValue);
        valueList.add(backLeftValue);
        valueList.add(backRightValue);

        return Collections.max(valueList);
    }

    public void setMotorDirection_Forward() {
        leftFront.setDirection(MotorConstants.REVERSE);
        leftRear.setDirection(MotorConstants.REVERSE);

        rightFront.setDirection(MotorConstants.FORWARD);
        rightRear.setDirection(MotorConstants.REVERSE);
    }

//    public String mostColor() {
//        red = coneSensor.red() / 1.15;
//        green = coneSensor.green() / 1.83;
//        blue = coneSensor.blue() / 1.43;
//        telemetry.addData("Red", red);
//        telemetry.addData("Green", green);
//        telemetry.addData("Blue", blue);
//        if (red > green && red > blue) {
//            return "Red";
//        } else if (green > blue) {
//            return "Green";
//        } else {
//            return "Blue";
//        }
//    }
}