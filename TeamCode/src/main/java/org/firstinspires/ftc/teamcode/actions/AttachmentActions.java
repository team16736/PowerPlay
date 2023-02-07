package org.firstinspires.ftc.teamcode.actions;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

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
    public Servo armExtender;
//    public Servo extender;
    public CRServo turnTable;
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
    private int error = 0;
    private int sum = 0;
    public boolean isDone = false;
    public double minVel = 10000;
    double startTime;
    int totalTicks;
    double ticksPerRevolution = 8192 * 96 / 100; // 7864
    double ticksPerDegree = ticksPerRevolution / 360; // 21.9

    boolean zeroStartBit = true;
    int zeroStartPos;

    boolean setBit = false;
    boolean lowerSetBit = false;

    boolean preset;


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
        armExtender = hardwareMap.get(Servo.class, ConfigConstants.ARM_EXTENDER);
//        extender = hardwareMap.get(Servo.class, ConfigConstants.EXTENDER);
        turnTable = hardwareMap.get(CRServo.class, ConfigConstants.TURN_TABLE);
        tableEncoder = hardwareMap.get(DcMotorEx.class, ConfigConstants.TURN_TABLE_ENCODER);
        scissorLift1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_ONE);
        scissorLift2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.SCISSOR_TWO);
//        elbowServo.setPosition(0.87);
        armExtender.setPosition(0.95);
//        extender.setPosition(1.0);
        gripperServo.setPosition(0.9);
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

    public void openGripper() { gripperServo.setPosition(0.9); }

    public void closeGripper() { gripperServo.setPosition(0.5); }

//    public void extendGripper(double distance) {
//        double maxDistance = 6.25;
//        extender.setPosition(1 - (distance / maxDistance));
//    }

    public void extendArm(double distance) {
        armExtender.setPosition(1-(distance / 22.1 + 0.05));
    }

    public void armPresets(Gamepad gamepad2) {
        if (gamepad2.dpad_down) {
            extendArm(1.3);
            preset = true;
        } else if (gamepad2.right_trigger > 0) {
            extendArm(2.58);
            preset = true;
        } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            extendArm(3.8);
            preset = true;
        } else if (gamepad2.dpad_up) {
            extendArm(5.3);
            preset = true;
        } else if (gamepad2.right_bumper) {
            extendArm(0.05);
            preset = true;
        } /*else if (Math.abs(gamepad2.right_trigger) > 0.01) {
            preset = false;
        }
        if (preset == false) {
            extendArm(gamepad2.right_trigger * 6.375);
        }*/
        telemetry.addData("preset", preset);
        telemetry.addData("trigger position", gamepad2.right_trigger);
        telemetry.addData("target servo position", armExtender.getPosition());
    }

    boolean selectMemBit;
    int selection;
    public void selectArmPreset(Gamepad gamepad) {
        if (gamepad.right_bumper && selectMemBit == true) {
            selection++;
            selectMemBit = false;
        } else if (!gamepad.right_bumper) {
            selectMemBit = true;
        }
    }

//    public boolean detectElement() {
//        if (junctionSensor.getDistance(DistanceUnit.CM) < 8) {
//            return true;
//        } else {
//            return false;
//        }
//    }

    public double getJunctionDistance() {
        return 1/*junctionSensor.getDistance(DistanceUnit.INCH)*/;
    }

//    public double getHorizontalFromDistance() {
//        double distance = junctionSensor.getDistance(DistanceUnit.INCH);
//        return Math.sin(77.5) * distance;
//    }
//
//    public double getVerticalFromDistance() {
//        double distance = junctionSensor.getDistance(DistanceUnit.INCH);
//        return Math.cos(77.5) * distance;
//    }
//
//    public double angleToJunction(double distance) {
//        double adjustedHorizontal = (Math.cos(77.5) * distance) - horizontalOffset + 0.5;
//        double adjustedVertical = (Math.sin(77.5) * distance) + verticalOffset;
//        return Math.atan(adjustedVertical / adjustedHorizontal);
//    }
//
//    public double finalDistanceToJunction(double distance) {
//        double adjustedHorizontal = (Math.cos(77.5) * distance) - horizontalOffset;
//        double adjustedVertical = (Math.sin(77.5) * distance) + verticalOffset;
//        double totalDistance = Math.sqrt(Math.pow(adjustedHorizontal, 2) + Math.pow(adjustedVertical, 2));
//        return totalDistance - (verticalOffset + grabberOffset);
//    }

    // need to tune encoder values
    public void turnTableEncoders(double degrees, boolean hasCone) {
//
//        Speed cap 0.2:
//        180:  ~0.0011,~0.0000000095 roughly
//        90:    0.0006, 0.000000065
//        20-45: 0.0022, 0.000000055
//        10:    0.0022, 0.0000005
//        5:	 0.0035, 0.00000052
        double kI;
        if (hasCone) {
            kI = 0.00000016;
        }
        else {
            kI = 0.0000002;
        }
        turnTableEncoders(degrees, 0.00044, kI, 0.5);
    }

    public void turnTableEncoders(double degrees, double Kp, double Ki, double speedCap) {
        totalTicks = (int) (ticksPerDegree * degrees);
        int velocityRange = 1;
        int acceptableError = (int) (ticksPerDegree * 2.1);

        if (!initBit) {
            isDone = false;
            prevTimeI = System.currentTimeMillis();
            startTime = prevTimeI;
            initBit = true;
        }

        error = totalTicks - tableEncoder.getCurrentPosition();

        velocity = tableEncoder.getVelocity();

        if ((Math.abs(error) < acceptableError && Math.abs(velocity) < velocityRange) || System.currentTimeMillis() - startTime > 6000) {
            isDone = true;
        }

        if (Math.abs(error) < (ticksPerDegree * 20)) {
            intervalI = System.currentTimeMillis() - prevTimeI;
            prevTimeI = System.currentTimeMillis();
            if (Math.abs(error) > acceptableError) {
                sum += (error * intervalI);
            }
        }

        P_Gain = error * Kp;
        I_Gain = sum * Ki;

        power = Range.clip(P_Gain + I_Gain, -1, 1);
        if (Math.abs(error) > acceptableError) {
            turnTable.setPower(power * speedCap);
        } else {
            turnTable.setPower(0);
            sum = 0;
        }

        //Not necessary anymore, deletable
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
            sum = 0;
            initBit = false;
        }
    }

    public boolean memBit90Degrees = true;
    double startPos90Deg;
    boolean left90Degrees;
    double turnAmount = 90.0;
    public void turnTable90Degrees(Gamepad gamepad) {
        if (gamepad.x) {
            left90Degrees = true;
            if (memBit90Degrees) {
                startPos90Deg = getTurntablePosition();
                memBit90Degrees = false;
            }
        } else if (gamepad.b) {
            left90Degrees = false;
            if (memBit90Degrees) {
                startPos90Deg = getTurntablePosition();
                memBit90Degrees = false;
            }
        }
        if (memBit90Degrees = false){
            if (left90Degrees) {
                turnAmount = -turnAmount;
            }
            turnTableEncoders(startPos90Deg + turnAmount, false);
            if (isDone) {
                memBit90Degrees = true;
            }
        }
        if (gamepad.right_stick_x != 0) {
            memBit90Degrees = true;
        }
    }

    public double getTurntablePosition() {
        return tableEncoder.getCurrentPosition() / ticksPerDegree;
    }

    public double getTurntableGoal() {
        return totalTicks / ticksPerDegree;
    }

    public void setLiftLevel(boolean low, boolean mid, boolean high) {
        if (!scissorLift1.isBusy()) {
            if (low) {
                liftScissor(6000, 700, true);
            } else if (mid) {
                liftScissor(6000, 1750, true);
            } else if (high) {
                liftScissor(6000, 4096, true);
            } else {
                scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
    public void setConeLevel(boolean low, boolean mid, boolean high) {
        if (!scissorLift1.isBusy()) {
            if (low) {
                liftScissor(6000, 220, true);
            } else if (mid) {
                liftScissor(6000, 260, true);
            } else if (high) {
                liftScissor(6000, 280, true);
            } else {
                scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void liftScissor(double speed, double verticalDistance, boolean hardCode) {
        int totalTicks = (int) -(0.1161 * Math.pow(verticalDistance, 3) - 2.2579 * Math.pow(verticalDistance, 2) + 56.226 * verticalDistance + 36.647);
        if (Math.abs(totalTicks) < 500) {
            scissorLift1.setTargetPositionTolerance(1);
            scissorLift2.setTargetPositionTolerance(1);
        } else {
            scissorLift1.setTargetPositionTolerance(5);
            scissorLift2.setTargetPositionTolerance(5);
        }
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

    public double speedf;
    public double posf;
    public double startposf;
    public boolean startf;
    public boolean liftToZero() {
        int currentPos = scissorLift1.getCurrentPosition();
//        RobotLog.dd("FindJunction", "Current Position %d, Current Velocity %f, start bit %b, starting position %d", currentPos, scissorLift1.getVelocity(), zeroStartBit, zeroStartPos);
        if (zeroStartBit) {
            zeroStartPos = currentPos;
            zeroStartBit = false;
            scissorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scissorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (currentPos > 0) {
            return false;
        }
        double speed = ((float) currentPos / (float) zeroStartPos) * 3000 + 200;
        scissorLift1.setVelocity(speed);
        scissorLift2.setVelocity(speed);
        speedf = speed;
        posf = currentPos;
        startf = zeroStartBit;
        startposf = zeroStartPos;
        return true;
    }

    public double getLiftHeight() {
        double horizontalDistance = -((scissorLift1.getCurrentPosition() + 68) / 145.1 * 0.314961);
        return 6 * Math.sqrt(39.69 - Math.pow(((12.6 - horizontalDistance) / 2), 2));
    }

    public void liftWithoutEncoders() {
        scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stayWhereSet(double joystickPosition) {
        if (Math.abs(joystickPosition) < 0.01) {
            if (!setBit) {
                liftScissor(3000, -scissorLift1.getCurrentPosition(), true);
                setBit = true;
            }
            if (!lowerSetBit && Math.abs(scissorLift1.getVelocity()) < 10) {
                liftScissor(3000, -scissorLift1.getCurrentPosition(), true);
                lowerSetBit = true;
            }
        } else {
            setBit = false;
            lowerSetBit = false;
        }
        telemetry.addData("target position", scissorLift1.getTargetPosition());
    }
}