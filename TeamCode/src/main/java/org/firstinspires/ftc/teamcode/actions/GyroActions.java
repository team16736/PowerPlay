package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.constants.MotorConstants;

public class GyroActions {
    DcMotorEx motorFrontL;
    DcMotorEx motorFrontR;
    DcMotorEx motorBackL;
    DcMotorEx motorBackR;
    private BNO055IMU imu = null;


    private double robotHeading = 0;
    public double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double leftFrontSpeed = 0;
    private double rightFrontSpeed = 0;
    private double leftBackSpeed = 0;
    private double rightBackSpeed = 0;
    int leftFrontTarget = 0;
    int leftBackTarget = 0;
    int rightFrontTarget = 0;
    int rightBackTarget = 0;

    double ticksPerInch = 31;
    double ticksPerInchStrafe = 39;

    public int driveState = 0;
    int distanceError;
    double adjSpeed;
    int totalTicks;

    public int strafeState = 0;

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
        motorFrontL.setDirection(MotorConstants.REVERSE);
        motorBackL.setDirection(MotorConstants.FORWARD);

        motorFrontR.setDirection(MotorConstants.REVERSE);
        motorBackR.setDirection(MotorConstants.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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

    public void encoderGyroDrive(double speed, double distance, double heading) {
        while (encoderGyroDriveStateMachine(speed, distance, heading)) {}
    }
    public boolean encoderGyroDriveStateMachine(double speed, double distance, double heading) {
        if (driveState == 0) {
            motorFrontL.setDirection(MotorConstants.REVERSE);
            motorBackL.setDirection(MotorConstants.FORWARD);

            motorFrontR.setDirection(MotorConstants.REVERSE);
            motorBackR.setDirection(MotorConstants.FORWARD);
            adjSpeed = speed;
            motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            totalTicks = (int) (ticksPerInch * distance);
            motorFrontL.setTargetPosition(totalTicks);
            motorFrontR.setTargetPosition(totalTicks);
            motorBackL.setTargetPosition(totalTicks);
            motorBackR.setTargetPosition(totalTicks);

            // Switch to RUN_TO_POSITION mode
            motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFrontL.setVelocity(speed);
            motorFrontR.setVelocity(speed);
            motorBackL.setVelocity(speed);
            motorBackR.setVelocity(speed);
            driveState = 1;
        }

        if (motorFrontL.isBusy()) {
            if (Math.abs(motorFrontL.getCurrentPosition()) > Math.abs(totalTicks * 0.9)) {
                adjSpeed = speed * 0.5;
            }
            headingError = getSteeringCorrection(heading, adjSpeed * 0.05, adjSpeed);
            if (distance < 0) {
                headingError *= -1;
            }
            motorFrontL.setVelocity(adjSpeed - headingError);
            motorFrontR.setVelocity(adjSpeed + headingError);
            motorBackL.setVelocity(adjSpeed - headingError);
            motorBackR.setVelocity(adjSpeed + headingError);

            distanceError = motorFrontL.getTargetPosition() - motorFrontL.getCurrentPosition();
            motorFrontL.setTargetPosition(motorFrontL.getCurrentPosition() + distanceError);
            motorFrontR.setTargetPosition(motorFrontR.getCurrentPosition() + distanceError);
            motorBackL.setTargetPosition(motorBackL.getCurrentPosition() + distanceError);
            motorBackR.setTargetPosition(motorBackR.getCurrentPosition() + distanceError);
            return true;
        } else {
            driveState = 0;
            return false;
        }
    }

    public void encoderGyroStrafe (double speed, double distance, double heading, boolean strafeLeft) {
        initEncoderGyroStrafeStateMachine(speed, distance, strafeLeft);
        while (encoderGyroStrafeStateMachine(speed, distance, heading, strafeLeft)) {}
    }
    public void initEncoderGyroStrafeStateMachine (double speed, double distance, boolean strafeLeft) {
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticksPerInch = 33.6;
        int totalTicks = (int) (ticksPerInch * distance);
        if (strafeLeft) {
            motorFrontL.setTargetPosition(-totalTicks);
            motorFrontR.setTargetPosition(totalTicks);
            motorBackL.setTargetPosition(totalTicks);
            motorBackR.setTargetPosition(-totalTicks);
        } else {
            motorFrontL.setTargetPosition(totalTicks);
            motorFrontR.setTargetPosition(-totalTicks);
            motorBackL.setTargetPosition(-totalTicks);
            motorBackR.setTargetPosition(totalTicks);
        }

        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontL.setVelocity(-speed);
        motorFrontR.setVelocity(-speed);
        motorBackL.setVelocity(-speed);
        motorBackR.setVelocity(-speed);
        strafeState = 1;
    }
    public boolean encoderGyroStrafeStateMachine (double speed, double distance, double heading, boolean strafeLeft) {
//        if (strafeState == 0) {
//            motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            double ticksPerInch = 33.6;
//            int totalTicks = (int) (ticksPerInch * distance);
//            if (strafeLeft) {
//                motorFrontL.setTargetPosition(-totalTicks);
//                motorFrontR.setTargetPosition(totalTicks);
//                motorBackL.setTargetPosition(totalTicks);
//                motorBackR.setTargetPosition(-totalTicks);
//            } else {
//                motorFrontL.setTargetPosition(totalTicks);
//                motorFrontR.setTargetPosition(-totalTicks);
//                motorBackL.setTargetPosition(-totalTicks);
//                motorBackR.setTargetPosition(totalTicks);
//            }
//
//            // Switch to RUN_TO_POSITION mode
//            motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            motorFrontL.setVelocity(-speed);
//            motorFrontR.setVelocity(-speed);
//            motorBackL.setVelocity(-speed);
//            motorBackR.setVelocity(-speed);
//            strafeState = 1;
//        }

        if (motorFrontL.isBusy()) {
            headingError = getSteeringCorrection(heading, speed * 0.05, speed);
            RobotLog.dd("FindJunction", "Heading Error %f", headingError / (speed * 0.05));
//            motorFrontL.setVelocity(-speed + headingError);
//            motorFrontR.setVelocity(-speed - headingError);
//            motorBackL.setVelocity(-speed + headingError);
//            motorBackR.setVelocity(-speed - headingError);
            if (!strafeLeft) {
                headingError *= -1;
            }
            motorFrontL.setVelocity(-speed - headingError);
            motorFrontR.setVelocity(-speed - headingError);
            motorBackL.setVelocity(-speed + headingError);
            motorBackR.setVelocity(-speed + headingError);

            distanceError = Math.abs(motorFrontL.getTargetPosition() - motorFrontL.getCurrentPosition());
            if (strafeLeft){
                motorFrontL.setTargetPosition(motorFrontL.getCurrentPosition() - distanceError);
                motorFrontR.setTargetPosition(motorFrontR.getCurrentPosition() + distanceError);
                motorBackL.setTargetPosition(motorBackL.getCurrentPosition() + distanceError);
                motorBackR.setTargetPosition(motorBackR.getCurrentPosition() - distanceError);
            }else{
                motorFrontL.setTargetPosition(motorFrontL.getCurrentPosition() + distanceError);
                motorFrontR.setTargetPosition(motorFrontR.getCurrentPosition() - distanceError);
                motorBackL.setTargetPosition(motorBackL.getCurrentPosition() - distanceError);
                motorBackR.setTargetPosition(motorBackR.getCurrentPosition() + distanceError);
            }
            telemetry.addData("is busy", true);
            return true;
        } else {
            strafeState = 0;
            telemetry.addData("isn't busy", false);
            return false;
        }
    }

    public void setVelocityStraight (double speed, double heading, int direction) {
        headingError = getSteeringCorrection(heading, speed * 0.05, speed);
        if (speed < 0) {
            headingError *= -1;
        }
        double fL;
        double fR;
        double bL;
        double bR;
        if (direction == HelperActions.BACKWARDS) {
            fL = -1;
            fR = -1;
            bL = -1;
            bR = -1;
        } else if (direction == HelperActions.RIGHT) {
            fL = 1;
            fR = -1;
            bL = -1;
            bR = 1;
        } else if (direction == HelperActions.LEFT) {
            fL = -1;
            fR = 1;
            bL = 1;
            bR = -1;
        } else {
            // Forwards
            fL = 1;
            fR = 1;
            bL = 1;
            bR = 1;
        }
        motorFrontL.setVelocity((speed - headingError) * fL);
        motorFrontR.setVelocity((speed + headingError) * fR);
        motorBackL.setVelocity((speed - headingError) * bL);
        motorBackR.setVelocity((speed + headingError) * bR);
    }
    public void gyroSpin(double speed,
                         double heading) {
        motorFrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
            telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }



    public boolean maintainHeading(double speed, double heading){
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

        return (Math.abs(headingError) > HEADING_THRESHOLD);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        return getSteeringCorrection(desiredHeading, proportionalGain, 1);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain, double range) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        telemetry.addData("robotHeading", robotHeading);
        telemetry.update();
//        opModeObj.sleep(3000);

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -range, range);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        motorFrontL.setPower(leftSpeed);
        motorBackL.setPower(leftSpeed);
        motorFrontR.setPower(rightSpeed);
        motorBackR.setPower(rightSpeed);
    }

    public void strafeRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftFrontSpeed = -drive - turn;
        rightFrontSpeed = drive + turn;
        leftBackSpeed = drive - turn;
        rightBackSpeed = -drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        motorFrontL.setPower(leftSpeed);
        motorBackL.setPower(leftSpeed);
        motorFrontR.setPower(rightSpeed);
        motorBackR.setPower(rightSpeed);
    }

    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void runUsingEncoders() {
        motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeObj.opModeIsActive()) {
            motorFrontL.setDirection(MotorConstants.REVERSE);
            motorBackL.setDirection(MotorConstants.FORWARD);
            motorFrontR.setDirection(MotorConstants.REVERSE);
            motorBackR.setDirection(MotorConstants.FORWARD);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * ticksPerInch);
            leftFrontTarget = motorFrontL.getCurrentPosition() + moveCounts;
            leftBackTarget = motorBackL.getCurrentPosition() + moveCounts;
            rightFrontTarget = motorFrontR.getCurrentPosition() + moveCounts;
            rightBackTarget = motorBackR.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            motorFrontL.setTargetPosition(leftFrontTarget);
            motorBackL.setTargetPosition(leftBackTarget);
            motorFrontR.setTargetPosition(rightFrontTarget);
            motorBackR.setTargetPosition(rightBackTarget);

            motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeObj.opModeIsActive() &&
                    (motorFrontL.isBusy() || motorFrontR.isBusy() || motorBackL.isBusy() || motorBackR.isBusy())) {
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                telemetry.update();
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              boolean moveLeft) {

        // Ensure that the opmode is still active
        if (opModeObj.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * ticksPerInchStrafe);

            if(moveLeft) {
                moveCounts *= -1.0;
            }

            leftFrontTarget = motorFrontL.getCurrentPosition() + moveCounts;
            leftBackTarget = motorBackL.getCurrentPosition() - moveCounts;
            rightFrontTarget = motorFrontR.getCurrentPosition() - moveCounts;
            rightBackTarget = motorBackR.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            motorFrontL.setTargetPosition(leftFrontTarget);
            motorBackL.setTargetPosition(leftBackTarget);
            motorFrontR.setTargetPosition(rightFrontTarget);
            motorBackR.setTargetPosition(rightBackTarget);

            motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeObj.opModeIsActive() &&
                    (motorFrontL.isBusy() || motorFrontR.isBusy() || motorBackL.isBusy() || motorBackR.isBusy())) {
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                telemetry.update();
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                if(moveLeft){
                    turnSpeed *= -1.0;
                }
                strafeRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      motorFrontL.getCurrentPosition(),
                    motorFrontR.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }
}
