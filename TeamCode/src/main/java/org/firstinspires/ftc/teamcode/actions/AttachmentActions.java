package org.firstinspires.ftc.teamcode.actions;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;

/**
 * Make sure to have the following:
 * <p>
 * 1. Hardware config
 * 2. Setup direction of motors
 * 3. Action method to do something (hookUpDown, drive, etc.,)
 * 4. Helper methods (stop, brake, leftTurn, rightTurn, etc.,)
 * <p>
 * Purpose: Drive the 4 wheels
 */
public class AttachmentActions {

    //    public Servo elbowServo;
    public Servo gripperServo;
    public Servo extender;
    public CRServo turnTable;
    public DistanceSensor junctionSensor;
    public ColorSensor boundaryDetector;
    public DcMotorEx scissorLift1;
    public DcMotorEx scissorLift2;
    public DcMotorEx tableEncoder;

    public double horizontalOffset = 3.25; //Left/right distance from sensor to center of rotation
    public double verticalOffset = 8.625; //Front/back distance from sensor to center of rotation, needs to be changed
    public double grabberOffset = 4.1; //Front/back distance from sensor to center of grabber, needs to be changed

    //Encoder Turntable stuff
    private boolean initBit = false;
    private double P_Gain = 0;
    private double I_Gain = 0;
    private double power = 0;
//  double intervalVelocity = 0;
//  double prevTimeVelocity = System.currentTimeMillis();
    private double intervalI = 0;
    private double prevTimeI = System.currentTimeMillis();
    private double velocity = 0;
    private int prevTicks = 0;
    private int error = 0;
    private int sum = 0;
    private int startingTicks = prevTicks;
    public boolean isDone = false;
    public double minVel = 10000;


    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    final float[] hsvValues = new float[3];

    /**
     * Creates a mecanum motor using the 4 individual motors passed in as the arguments
     *
     * @param opModeTelemetry : Telemetry to send messages to the Driver Control
     * @param opModeHardware  : Hardware Mappings
     */
    // Constructor
    public AttachmentActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {

        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        // 1. Hardware config
//        elbowServo = hardwareMap.get(Servo.class, ConfigConstants.ELBOW_SERVO);
        gripperServo = hardwareMap.get(Servo.class, ConfigConstants.GRIPPER_SERVO);
        extender = hardwareMap.get(Servo.class, ConfigConstants.EXTENDER);
        turnTable = hardwareMap.get(CRServo.class, ConfigConstants.TURN_TABLE);
        tableEncoder = hardwareMap.get(DcMotorEx.class, ConfigConstants.TURN_TABLE_ENCODER);
        junctionSensor = hardwareMap.get(DistanceSensor.class, ConfigConstants.JUNCTION_DETECTOR);
        boundaryDetector = hardwareMap.get(ColorSensor.class, ConfigConstants.BOUNDARY_DETECTOR);
        scissorLift1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_ONE);
        scissorLift2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_TWO);
//        elbowServo.setPosition(0.87);
        extender.setPosition(1.0);
        gripperServo.setPosition(0.8);
        turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
        turnTable.setPower(0.0);
        scissorLift1.setDirection(DcMotorEx.Direction.REVERSE);
        scissorLift2.setDirection(DcMotorEx.Direction.REVERSE);
        scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tableEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int tableencodercount() {
        return tableEncoder.getCurrentPosition();
    }

    public void openGripper() {
        gripperServo.setPosition(0.8);
    }

    public void closeGripper() {
        gripperServo.setPosition(0.3);
    }

    public void extendGripper(double distance) {
        double maxDistance = 6.25;
        extender.setPosition(1 - (distance / maxDistance));
    }

    public boolean detectElement() {
        if (junctionSensor.getDistance(DistanceUnit.CM) < 8) {
            return true;
        } else {
            return false;
        }
    }

    public double getJunctionDistance() {
        return junctionSensor.getDistance(DistanceUnit.INCH);
    }

    public double getHorizontalFromDistance() {
        double distance = junctionSensor.getDistance(DistanceUnit.INCH);
        return Math.sin(77.5) * distance;
    }

    public double getVerticalFromDistance() {
        double distance = junctionSensor.getDistance(DistanceUnit.INCH);
        return Math.cos(77.5) * distance;
    }

    public double angleToJunction(double distance) {
        double adjustedHorizontal = (Math.cos(77.5) * distance) - horizontalOffset;
        double adjustedVertical = (Math.sin(77.5) * distance) + verticalOffset;
        return Math.atan(adjustedVertical / adjustedHorizontal);
    }

    public double finalDistanceToJunction(double distance) {
        double adjustedHorizontal = (Math.cos(77.5) * distance) - horizontalOffset;
        double adjustedVertical = (Math.sin(77.5) * distance) + verticalOffset;
        double totalDistance = Math.sqrt(Math.pow(adjustedHorizontal, 2) + Math.pow(adjustedVertical, 2));
        return totalDistance - (verticalOffset + grabberOffset);
    }

    // need to tune encoder values
    public void turnTableEncoders(double degrees, double speed) {
//        20-90: 0.0022, 0.000000055
//        10:    0.0022, 0.0000005
//        5:	 0.0035, 0.00000052
        turnTableEncoders(degrees, speed, 0.0022, 0.0000005);
    }

    public void turnTableEncoders(double degrees, double speed, double Kp, double Ki) {
        double ticksPerRevolution = 8192 * 96 / 100; // 7864
        double ticksPerDegree = ticksPerRevolution / 360;
        int totalTicks = (int) (ticksPerDegree * degrees);
        int velocityRange = -1;
        int acceptableError = (int) ticksPerDegree * 1;

        if (!initBit) {
            isDone = false;
            initBit = true;
        }

        error = tableEncoder.getCurrentPosition() - totalTicks;

        velocity = tableEncoder.getVelocity();

        if (Math.abs(error) < acceptableError && Math.abs(velocity) < velocityRange) {
            isDone = true;
        }

        intervalI = System.currentTimeMillis() - prevTimeI;
        prevTimeI = System.currentTimeMillis();
        sum += (error * intervalI);

        P_Gain = error * Kp;
        I_Gain = sum * Ki;

        power = Range.clip(P_Gain + I_Gain, -1, 1);
        turnTable.setPower(-(power * speed));

        if (Math.abs(error) < acceptableError) {
            if (velocity < minVel) {minVel = velocity;}
        }

        telemetry.addData("Position", tableEncoder.getCurrentPosition());
        telemetry.addData("Error", error);
        telemetry.addData("Sum", sum);
        telemetry.addData("P_Gain", P_Gain);
        telemetry.addData("I_Gain", I_Gain);
        telemetry.addData("Is Done", isDone);
        telemetry.update();
        if (isDone) {
            P_Gain = 0;
            I_Gain = 0;
            power = 0;
            intervalI = 0;
            prevTimeI = System.currentTimeMillis();
            velocity = 0;
            prevTicks = tableEncoder.getCurrentPosition();
            error = 0;
            sum = 0;
            startingTicks = prevTicks;
            initBit = false;
        }
    }

    public double getTurntablePosition() {
        double ticksPerRevolution = 8192 * 96 / 100; // 7864
        double ticksPerDegree = ticksPerRevolution / 360;
        return tableEncoder.getCurrentPosition() / ticksPerDegree;
    }

    public void setLiftLevel(boolean low, boolean mid, boolean high) {
        if (!scissorLift1.isBusy()) {
            if (low) {
                liftScissor(6000, 700, true);
            } else if (mid) {
                liftScissor(6000, 1750, true);
            } else if (high) {
                liftScissor(6000, 3900, true);
            } else {
                scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void liftScissor(double speed, double verticalDistance, boolean hardCode) {
        double horizontalDistance = 12.6 - (2 * Math.sqrt(39.69 - Math.pow(verticalDistance / 6, 2)));
        int totalTicks = (int) (-68 + (-horizontalDistance / 0.314961 * 145.1));
        if (hardCode) {
            totalTicks = (int) -verticalDistance;
        }
        scissorLift1.setTargetPosition(totalTicks);
        scissorLift2.setTargetPosition(totalTicks);

        scissorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        scissorLift1.setVelocity(speed);
        scissorLift2.setVelocity(speed);
    }

    public double getLiftHeight() {

        double horizontalDistance = -((scissorLift1.getCurrentPosition() + 68) / 145.1 * 0.314961);
        return 6 * Math.sqrt(39.69 - Math.pow(((12.6 - horizontalDistance) / 2), 2));
    }

    public void liftWithoutEncoders() {
        scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}