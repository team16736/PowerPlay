package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    public double THROTTLE = 0.5;

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_LEFT);
        leftRear = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_LEFT);

        rightFront = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_RIGHT);
        rightRear = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_RIGHT);

        setMotorDirection_Forward();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive(
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * 0.5),      //joystick controlling strafe
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * 0.5),     //joystick controlling forward/backward
                    (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * 0.5));    //joystick controlling rotation
        }
    }
    public void drive(double speedX, double speedY, double rotation){

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
        leftRear.setDirection(MotorConstants.FORWARD);

        rightFront.setDirection(MotorConstants.REVERSE);
        rightRear.setDirection(MotorConstants.FORWARD);
    }
}