package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class GyroActions{
    DcMotorEx motorFrontL;
    DcMotorEx motorFrontR;
    DcMotorEx motorBackL;
    DcMotorEx motorBackR;
    private BNO055IMU imu         = null;

    private double          robotHeading      = 0;
    private double          headingOffset     = 0;
    private double          headingError      = 0;
    private double          targetHeading     = 0;
    static final double     HEADING_THRESHOLD = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN       = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN      = 0.03;     // Larger is more responsive, but also less stable
    private double          driveSpeed        = 0;
    private double          turnSpeed         = 0;
    private double          leftSpeed         = 0;
    private double          rightSpeed        = 0;

    private static LinearOpMode opModeObj;



    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();

    public GyroActions(LinearOpMode opMode, Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        opModeObj = opMode;
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;
        motorFrontL = hardwareMap.get(DcMotorEx.class, "leftFront");
        motorFrontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        motorBackL = hardwareMap.get(DcMotorEx.class, "leftRear");
        motorBackR = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Probably necessary
        motorFrontL.setDirection(DcMotor.Direction.REVERSE);
        motorBackL.setDirection(DcMotor.Direction.REVERSE);
        motorFrontR.setDirection(DcMotor.Direction.FORWARD);
        motorBackR.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Probably necessary
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetHeading();
    }
    public void gyroSpin(double speed,
                            double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeObj.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            telemetry.addData("robotHeading Error 1", robotHeading);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -speed, speed);
            telemetry.addData("robotHeading Error 2", robotHeading);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            telemetry.addData("robotHeading Error 3", robotHeading);

            // Display drive status for the driver.
            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
            telemetry.addData("robotHeading Error 4", robotHeading);
            telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
            telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        telemetry.addData("robotHeading", robotHeading);
        telemetry.update();
//        opModeObj.sleep(3000);

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        motorFrontL.setPower(leftSpeed);
        motorBackL.setPower(leftSpeed);
        motorFrontR.setPower(rightSpeed);
        motorBackR.setPower(rightSpeed);
    }
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    public void runUsingEncoders(){
        motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
