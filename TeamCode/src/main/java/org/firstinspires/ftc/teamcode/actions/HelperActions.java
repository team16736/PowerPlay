package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class HelperActions extends LinearOpMode {

    protected ColorSensor right_sensor;
    protected ColorSensor left_sensor;
    protected boolean foundStone = false;
    protected float hsvValues[] = {0F,0F,0F};

    public final double SPEED = 0.5;

    public static int LEFT = 1;
    public static int RIGHT = 2;
    public static int FORWARDS = 3;
    public static int BACKWARDS = 4;

    private int speeding = 0;
    private double speed = 0.8;
    private int speedingArm = 0;
    private double speedArm = 0.6;

    public void drive_ForwardAndStop(DriveActions driveActions, double speed, double drivingTime) {
        driveActions.setMotorDirection_Forward();
        driveActions.driveByTime(this, speed, drivingTime);
        driveActions.stop();
    }

    public boolean strafeAndDetect(EncoderActions encoderActions, AttachmentActions attachmentActions, double speed, double distance, boolean strafeLeft){
        encoderActions.motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderActions.motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderActions.motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderActions.motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor's target position to 6.4 rotations
        double ticksPerInch = 64.75;
        int totalTicks = (int) (ticksPerInch * distance);
        if (strafeLeft){
            encoderActions.motorFrontL.setTargetPosition(-totalTicks);
            encoderActions.motorFrontR.setTargetPosition(totalTicks);
            encoderActions.motorBackL.setTargetPosition(totalTicks);
            encoderActions.motorBackR.setTargetPosition(-totalTicks);
        }else{
            encoderActions.motorFrontL.setTargetPosition(totalTicks);
            encoderActions.motorFrontR.setTargetPosition(-totalTicks);
            encoderActions.motorBackL.setTargetPosition(-totalTicks);
            encoderActions.motorBackR.setTargetPosition(totalTicks);
        }


        // Switch to RUN_TO_POSITION mode
        encoderActions.motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 1 revolution per second
        encoderActions.motorFrontL.setVelocity(-speed);
        encoderActions.motorFrontR.setVelocity(-speed);
        encoderActions.motorBackL.setVelocity(-speed);
        encoderActions.motorBackR.setVelocity(-speed);

        //motorFrontL.isBusy()hile the Op Mode is running, show the motor's status via telemetry
        while (encoderActions.motorFrontL.isBusy()) {
            telemetry.addData("FL is at target", !encoderActions.motorFrontL.isBusy());
            telemetry.addData("FR is at target", !encoderActions.motorFrontR.isBusy());
            telemetry.addData("BL is at target", !encoderActions.motorBackL.isBusy());
            telemetry.addData("BR is at target", !encoderActions.motorBackR.isBusy());
            telemetry.update();
            if (attachmentActions.detectElement()){
                if (attachmentActions.detectElement()){
                    return true;
                }
            }
        }
        return false;
    }
//    public void fancySpinRight(EncoderActions encoderActions, double speed, double distance){
//        encoderActions.fancySpin(speed, distance, false);
//    }
//    public void fancySpinLeft(EncoderActions encoderActions, double speed, double distance){
//        encoderActions.fancySpin(speed, distance, true);
//    }
    public int elementDetection(EncoderActions encoderActions, AttachmentActions attachmentActions, boolean strafeLeft) {
        if (attachmentActions.detectElement()) {
            return 1;
        } else if (strafeAndDetect(encoderActions, attachmentActions, 762.2, 11, strafeLeft)) {
            return 2;
        } else {
            return 3;
        }
    }
    public void changeSpeed(DriveActions driveActions, boolean upOne, boolean downOne, boolean upTwo, boolean downTwo){
        if(upOne){
            speeding++;
            if(speeding == 1){
                speed = speed + 0.1;
            }
        }
        if(downOne){
            speeding++;
            if(speeding == 1){
                speed = speed - 0.1;
            }
        }
        if(upTwo){
            speeding++;
            if(speeding == 1){
                speed = speed + 0.2;
            }
        }
        if(downTwo){
            speeding++;
            if(speeding == 1){
                speed = speed - 0.2;
            }
        }
        if(!upOne && !downOne && !upTwo && !downTwo){
            speeding = 0;
        }
        if (speed < 0){
            speed = 0;
        }
        if (speed > 1.0){
            speed = 1.0;
        }
        driveActions.setSpeed(speed);
        telemetry.addData("speed: ", speed);
    }
    public double changeSpeedArm(boolean up, boolean down) {
        if (up) {
            speedingArm++;
            if (speedingArm == 1) {
                speedArm = speedArm + 0.1;
            }
        }
        if (down) {
            speedingArm++;
            if (speedingArm == 1) {
                speedArm = speedArm - 0.1;
            }
        }
        if (!up && !down) {
            speedingArm = 0;
        }
        if (speedArm < 0) {
            speedArm = 0;
        }
        if (speedArm > 1.0) {
            speedArm = 1.0;
        }
        return speedArm;
    }
}