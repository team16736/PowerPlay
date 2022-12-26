package org.firstinspires.ftc.teamcode.actions;//package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FindJunctionAction {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DriveActions driveActions;
    private AttachmentActions attachmentActions;
    private DistanceSensorActions s1;
    private EncoderActions encoderActions;
    private static LinearOpMode opModeObj;

    public FindJunctionAction(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, DriveActions driveActions, AttachmentActions attachmentActions, DistanceSensorActions distanceSensorActions, EncoderActions encoderActions) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.driveActions = driveActions;
        this.attachmentActions = attachmentActions;
        this.s1 = distanceSensorActions;
        this.encoderActions = encoderActions;
        opModeObj = opMode;
    }

    public int findJunction(double distance, double scissorDistance) {
        //Telemetry telemetry;
        HardwareMap hardwareMap;
        ElapsedTime runtime = new ElapsedTime();
        driveActions.setMotorDirection_Forward();

        encoderActions.resetEncoder();
        
/*
        double[] distances = new double[1000];
        double[] times = new double[1000];
        int i = 0;
*/

        boolean memBitOn = false;
        boolean memBitOff = false;
        int ticks = 0;
        int ticksOff = 0;
        double targetPos = distance * 32.3;
        double pwr = 1.0;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        double speed = 762.2;
        double degrees = -20;
        double topSpeed = 2400 * 0.5; // = 1200
        /*    
        motorFrontL.setTargetPosition(targetPos);
        motorFrontR.setTargetPosition(targetPos);
        motorBackL.setTargetPosition(targetPos);
        motorBackR.setTargetPosition(targetPos);   
        
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        encoderActions.motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderActions.motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderActions.motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderActions.motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        /*
        motorFrontL.setPower(pwr);
        motorFrontR.setPower(pwr);
        motorBackL.setPower(pwr);
        motorBackR.setPower(pwr);
        */

        double ramp = 0.0;
        double minSpeed = 0.05 * topSpeed;
        double transitionVelocity = 0.0;

        double dur = 0.0;
        double adj = 0.25;

        double dist = 2000.0;
        int ticksAtLowestDist = 0;

        int counter = 0;
        //------------------------------
        int totalTicks = (int) -(0.1161 * Math.pow(scissorDistance, 3) - 2.2579 * Math.pow(scissorDistance, 2) + 56.226 * scissorDistance + 36.647);
        //if (hardCode) {
        //    totalTicks = (int) -verticalDistance;

        attachmentActions.liftScissor(3000, -totalTicks, true);
        //-------------------------------------

        while (encoderActions.motorFrontL.getCurrentPosition() < targetPos && opModeObj.opModeIsActive()) {

            if (encoderActions.motorFrontL.getCurrentPosition() < (targetPos * adj)) {
                encoderActions.velocity(topSpeed, topSpeed, topSpeed, topSpeed);
                transitionVelocity = encoderActions.motorFrontL.getVelocity();
            } else {
                //ramp = topSpeed * 2 * ((targetPos-motorFrontL.getCurrentPosition())/targetPos);
                ramp = topSpeed * (targetPos - encoderActions.motorFrontL.getCurrentPosition()) / (targetPos - (targetPos * adj));
                if (ramp < minSpeed) {
                    ramp = minSpeed;
                }
                encoderActions.velocity(ramp, ramp, ramp, ramp);
            }
            

            double distanceS1 = s1.getSensorDistance(DistanceUnit.MM);
            int currentPos = encoderActions.motorFrontL.getCurrentPosition();
            if (!attachmentActions.scissorLift1.isBusy() && (targetPos - currentPos) < 600 && distanceS1 < 2000) {


                if (distanceS1 < dist) {

                    dist = distanceS1;
                    ticksAtLowestDist = currentPos;

                }

                if (distanceS1 < 200) {

                    memBitOn = true;

                }

                if ((memBitOn = true) && (distanceS1 > dist)) {

                    counter = counter + 1;

                } else {
                    counter = 0;
                }

                if (counter > 2) {

                    targetPos = ticksAtLowestDist;

                    encoderActions.velocity(0, 0, 0, 0);

                }
            /*
                distances[i] = distSensor.getDistance(DistanceUnit.MM);
                times[i] = System.currentTimeMillis();
                i++;
                */

                telemetry.addData("Scissor Y", attachmentActions.scissorLift1.getCurrentPosition());
                telemetry.addData("Scissor Y Target", totalTicks);
                telemetry.update();

            }

            // telemetry.addData("ramp", ramp);
            // telemetry.addData("Current pos", motorFrontL.getCurrentPosition());
            // telemetry.addData("topSpeed", topSpeed);
            // telemetry.addData("targetPos", targetPos);
            // telemetry.addData("switch velocity", transitionVelocity);
            // telemetry.addData("distance", dist);
            // telemetry.addData("Scissor Y", scissor1.getCurrentPosition());
            // telemetry.addData("Scissor Y Target", totalTicks);
            // telemetry.update();

        }

        encoderActions.velocity(0, 0, 0, 0);

        dur = runtime.time();
            
        
        /*
        double topSpeed =0.0;
        while(motorFrontL.isBusy() && motorFrontR.isBusy() && motorBackL.isBusy() && motorBackR.isBusy()){
            if (motorFrontL.getVelocity() > topSpeed) {
                topSpeed = motorFrontL.getVelocity();
            }
        }
        */
        
        /*
        for(int j = 0; j < i; j++) {
            double time = times[j];
            if(j > 0) {
                time = times[j] - times[j-1];
            }
            String distance = "f";
            distance = distance.valueOf(distances[j]);
            telemetry.addData("time", time);
            telemetry.addData("distance", distances[j]);
        }
        */

        telemetry.addData("distance", dist);
        telemetry.addData("Motor ticks at lowest dist", ticksAtLowestDist);
        telemetry.addData("Current pos", encoderActions.motorFrontL.getCurrentPosition());
        telemetry.addData("Distance past lowest dist", (encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist));
        telemetry.addData("time", dur);
        telemetry.addData("counter", counter);
        telemetry.update();
        return encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist;
    }
}