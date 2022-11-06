package org.firstinspires.ftc.teamcode.actions;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;

import java.util.Base64;

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
    public CRServo turnTable;
    public DistanceSensor elementDetector;
    public ColorSensor boundaryDetector;
    public DcMotorEx scissorLift1;
    public DcMotorEx scissorLift2;
    public DcMotorEx tableEncoder;


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
        turnTable = hardwareMap.get(CRServo.class, ConfigConstants.TURN_TABLE);
        tableEncoder = hardwareMap.get(DcMotorEx.class, ConfigConstants.TURN_TABLE_ENCODER);
        elementDetector = hardwareMap.get(DistanceSensor.class, ConfigConstants.ELEMENT_DETECTOR);
        boundaryDetector = hardwareMap.get(ColorSensor.class, ConfigConstants.BOUNDARY_DETECTOR);
        scissorLift1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_ONE);
        scissorLift2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_TWO);
//        elbowServo.setPosition(0.87);
        gripperServo.setPosition(0.65);
        turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
        turnTable.setPower(0.0);
        scissorLift1.setDirection(DcMotorEx.Direction.REVERSE);
        scissorLift2.setDirection(DcMotorEx.Direction.REVERSE);
        tableEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int tableencodercount() {
        return tableEncoder.getCurrentPosition();
    }

    public void contractElbow() {
    } //delete 72-74

    public void elbowLevel1() {
    }

    public void elbowLevel2() {
    }

    public void openGripper() {
        gripperServo.setPosition(0.6);
    }

    public void closeGripper() {
        gripperServo.setPosition(0.3);
    }

    public boolean detectElement() {
        if (elementDetector.getDistance(DistanceUnit.CM) < 8) {
            return true;
        } else {
            return false;
        }
    }

    // need to tune encoder values
    public void turnTableEncoders(double degrees, double speed){
        turnTableEncoders(degrees, speed, 0.0015, 0.00001);
    }
    public void turnTableEncoders(double degrees, double speed, double Kp, double Ki) {
        double ticksPerRevolution = 8192;
        double ticksPerDegree = ticksPerRevolution / 360;
        int totalTicks = (int) (ticksPerDegree * degrees);

        double P_Gain = 0;
        double I_Gain = 0;
        double power = 0;
        double prevTime = System.currentTimeMillis();
        int prevTicks = tableEncoder.getCurrentPosition();
        int velocity = 0;
        int velocityRange = 10;
        int acceptableError = (int) ticksPerDegree * 1;
        int error = tableEncoder.getCurrentPosition() - totalTicks;
        int sum = error;
        int startingTicks = prevTicks;
        int distance = tableEncoder.getCurrentPosition()-startingTicks;
        int maxDistance = distance;
        boolean isDone = false;

        while (!isDone){
            error = tableEncoder.getCurrentPosition() - totalTicks;

            if((System.currentTimeMillis()-prevTime)>9){
                telemetry.addData("interval", System.currentTimeMillis()-prevTime);
                velocity = (tableEncoder.getCurrentPosition() - prevTicks);
                prevTicks = tableEncoder.getCurrentPosition();
                prevTime = System.currentTimeMillis();
            }

            if(Math.abs(error) < acceptableError && velocity < velocityRange){
                isDone = true;
            }

            // for tuning purposes
            distance = tableEncoder.getCurrentPosition() - startingTicks;
            if (distance > maxDistance){
                maxDistance = distance;
            }

            sum += error;

            P_Gain = error * Kp;
            I_Gain = sum * Ki;

            power = Range.clip(P_Gain + I_Gain, -1, 1);
            turnTable.setPower(-(power * speed));

            telemetry.addData("Position", tableEncoder.getCurrentPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Sum", sum);
            telemetry.addData("P_Gain", P_Gain);
            telemetry.addData("I_Gain", I_Gain);
            telemetry.addData("Is Done", isDone);
            telemetry.addData("max distance", maxDistance);
            telemetry.update();
        }
        turnTable.setPower(0);
    }

    //    public void spinSlide(double speed, double degrees){
//        slideTurnMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        double ticksPerRevolution = 5281.1;
//        double ticksPerDegree = (ticksPerRevolution)/360;
//        int totalTicks = (int) (ticksPerDegree * degrees);
//
//        slideTurnMotor.setTargetPosition(totalTicks);
//
//        slideTurnMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        slideTurnMotor.setVelocity(speed);
//    }
//
//    public void extendSlide(double distance){
//        double maxPosition = -3610;
//        double minPosition = 0;
//        double maxLength = 24;
//        double ticksPerInch = (maxPosition-minPosition)/maxLength;
//        int totalTicks = (int) (ticksPerInch * distance);
//
//        slideExtendMotor.setTargetPosition(totalTicks);
//
//        slideExtendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        slideExtendMotor.setPower(0.5);
//    }
//    public boolean isSlideExtendMotorBusy(){
//        return slideExtendMotor.isBusy();
//    }
//    public boolean isSlideRotateMotorBusy(){
//        return  slideTurnMotor.isBusy();
//    }
//    public void adjustSlide(double speed){}
//    public void teleOpSlideRotate(double speed, int distance){
//        int currentTicks = slideTurnMotor.getCurrentPosition();
//        int totalTicks = (int) currentTicks + distance;
//        slideTurnMotor.setTargetPosition(totalTicks);
//
//        //Switch to RUN_TO_POSITION mode
//        slideTurnMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        //Start the motor moving by setting the max velocity to 1 revolution per second
//        slideTurnMotor.setVelocity(speed);
//    }
    public void extendSlide(double distance, double speed) {
        scissorLift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public boolean detectBarrier() {
        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.RGBToHSV(boundaryDetector.red() * 8, boundaryDetector.green() * 8, boundaryDetector.blue() * 8, hsvValues);

        telemetry.addLine()
                .addData("Red", boundaryDetector.red())
                .addData("Green", boundaryDetector.green())
                .addData("Blue", boundaryDetector.blue());
        telemetry.addLine()
                .addData("Hue", hsvValues[0])
                .addData("Saturation", hsvValues[1])
                .addData("Value", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if ((boundaryDetector.red() > 60) || (boundaryDetector.green() > 60) || (boundaryDetector.blue() > 60)) {
            telemetry.addData("Tape", " ");
            return true;
        } else {
            telemetry.addData("No Tape", " ");
            return false;
        }
    }
}