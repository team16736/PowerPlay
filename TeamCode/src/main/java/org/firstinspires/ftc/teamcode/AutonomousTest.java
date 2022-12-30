package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.FindJunctionAction;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private FindImageOnCone findImageOnCone = null;
    private DistanceSensorActions s1 = null;

    int state = 0;
    double startTime;
    public int coneNum = 5;

    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        s1 = new DistanceSensorActions(hardwareMap, 0.5, 10, ConfigConstants.BASE_RANGE);
        driveActions.setMotorDirection_Forward();
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FindJunctionAction findJunctionAction = new FindJunctionAction(hardwareMap, telemetry, this, driveActions, attachmentActions, s1, encoderActions, gyroActions);

        boolean memBitOn = false;
        boolean memBitOff = false;
        int ticks = 0;
        int ticksOff = 0;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

//        while (opModeIsActive()) {
        if (opModeIsActive()) {
            double speed = 762.2;
            double degrees = -20;
            double prevTime = System.currentTimeMillis();

//            attachmentActions.turnTableEncoders(0, false);
//            gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//            while (!attachmentActions.isDone || findJunctionAction.state != 0) {
//                attachmentActions.turnTableEncoders(0, false);
//                if (gyroActions.driveState != 0) {
//                    gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//                }
//            }


            attachmentActions.closeGripper();
            sleep(500);
            attachmentActions.liftScissor(3000, 10, false);
            encoderActions.encoderStrafe(400, 6, false);
            String location = findImageOnCone.findObject();
            findJunctionAction.findJunction(47, 24, false, FORWARDS);
            attachmentActions.liftScissor(3000, 260, true);
            attachmentActions.openGripper();
            gyroActions.encoderGyroDriveStateMachine(700, 10, 0);
            attachmentActions.turnTableEncoders(180, false);
            while (gyroActions.encoderGyroDriveStateMachine(700, 10, 0)) {
                attachmentActions.turnTableEncoders(180, false);
            }
            gyroActions.encoderGyroDriveStateMachine(700, -1, 0);
            attachmentActions.turnTableEncoders(180, false);
            while (gyroActions.encoderGyroDriveStateMachine(700, -1, 0)) {
                attachmentActions.turnTableEncoders(180, false);
            }
            double previousTime = System.currentTimeMillis();
            while (System.currentTimeMillis()-previousTime < 100) {
                attachmentActions.turnTableEncoders(180, false);
            }
            encoderActions.encoderStrafeNoWhile(700, 20, true);
            while (encoderActions.motorFrontL.isBusy()) {
                attachmentActions.turnTableEncoders(180, false);
            }
            attachmentActions.closeGripper();
            sleep(500);
            attachmentActions.liftScissor(3000, 10, false);
            coneNum--;
            while (attachmentActions.getLiftHeight() < 9){}
            findJunctionAction.findJunctionStateMachine(40, 26, false, false, RIGHT);
            attachmentActions.turnTableEncoders(90, false);
            while (findJunctionAction.state != 0) {
                attachmentActions.turnTableEncoders(90, false);
                findJunctionAction.findJunctionStateMachine(40, 26, false, false, RIGHT);
            }
            placeCone(attachmentActions, findJunctionAction, encoderActions);
            placeCone(attachmentActions, findJunctionAction, encoderActions);
            moveToLocation(gyroActions, location);
            telemetry.addData("time", System.currentTimeMillis() - prevTime);
            telemetry.update();
            sleep(10000);


//            James debug code
//            attachmentActions.closeGripper();
//            sleep(500);
//            findJunctionAction.findJunction(48,24, false, RIGHT);
//
//            The static code for goToCone
//            encoderActions.encoderSpinNoWhile(300, -90, true);
//            sleep(500);
//            attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            attachmentActions.liftScissor(3000, 290, true);
//            attachmentActions.turnTableEncoders(-90, false);
//            while (!attachmentActions.isDone) {
//                attachmentActions.turnTableEncoders(-90, false);
//            }
//            gyroActions.encoderGyroDrive(700, 47.5, -90);
//            attachmentActions.closeGripper();
//            sleep(500);
//            attachmentActions.liftScissor(3000, 24, false);
//            attachmentActions.turnTableEncoders(0, false);


//            sleep(3000);
//            encoderActions.encoderSpin(300, 90, false);
//            for (int i = 0; i < 5; i++) {
////                encoderActions.encoderStrafe(700, 40, false);
//                gyroActions.encoderGyroDrive(700, 60, 0);
//                sleep(500);
////                encoderActions.encoderStrafe(700, 40, true);
//                gyroActions.encoderGyroDrive(700, -40, -90);
//                sleep(500);
//            }



            /*if(!memBitOff) {
                attachmentActions.turnTable.setPower(0.1);
                memBitOff = true;
            }
            if(attachmentActions.getJunctionDistance() < 200 && !memBitOn){
                ticks = attachmentActions.tableencodercount();
                memBitOn = true;
                telemetry.addData("ticks on", ticks);
                telemetry.update();
            }
            if(attachmentActions.getJunctionDistance() > 200 && memBitOn){
                ticksOff = attachmentActions.tableencodercount();
                memBitOn = false;
                attachmentActions.turnTable.setPower(0);
                telemetry.addData("ticks off", ticksOff);
                telemetry.update();
            }*/
//            attachmentActions.extendGripper(5);
//            while (attachmentActions.getJunctionDistance() < 1000) {}

//            telemetry.addData("distance", attachmentActions.getJunctionDistance());
//            telemetry.addData("ticks on", ticks);
//            telemetry.addData("ticks off", ticksOff);
//            telemetry.addData("difference", ticks - ticksOff);
//            telemetry.update();
//            gyroActions.gyroSpin(0.2, 90.0);
        }
    }

    private void placeCone(AttachmentActions attachmentActions, FindJunctionAction findJunctionAction, EncoderActions encoderActions) {
        attachmentActions.liftScissor(3000, 1500, true);
        sleep(100);
        double strafeDistance = 34;
        if (coneNum == 3 || coneNum == 4) {strafeDistance = 36;}
        int strafeSpeed = 650;
        gyroActions.encoderGyroDriveStateMachine(700, -1, 0);
        while (gyroActions.encoderGyroDriveStateMachine(700, -1, 0)) {
            attachmentActions.turnTableEncoders(180, false);
        }
        gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, true);
        //Run turntable first bc cannot run both(would hit junction) and turntable is slower than lowering the lift
        while (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < 235) {
            attachmentActions.turnTableEncoders(180, false);
            gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, true);
        }
        int grabHeight = 220;
        if (coneNum == 4) {grabHeight = 240;}
        attachmentActions.liftScissor(3000, grabHeight, true);
        attachmentActions.openGripper();
        while (gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, true)) {
            attachmentActions.turnTableEncoders(180, false);
        }
        attachmentActions.closeGripper();
        sleep(500);
        coneNum--;
        attachmentActions.liftScissor(3000, 10, false);
        while (attachmentActions.getLiftHeight() < 9){}
        findJunctionAction.findJunctionStateMachine(40, 26, false, false, RIGHT, -1);
        attachmentActions.turnTableEncoders(90, false);
        while (findJunctionAction.state != 0) {
            attachmentActions.turnTableEncoders(90, false);
            findJunctionAction.findJunctionStateMachine(40, 26, false, false, RIGHT, -1);
        }
    }
    private void moveToLocation(GyroActions gyroActions, String location) {
        if (location == "Racket") {
            //            location 3
            gyroActions.encoderGyroDrive(2500, 1, 0);
            sleep(100);
            gyroActions.encoderGyroStrafe(2300, 8, 0, false);
            telemetry.addData(location, "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            gyroActions.encoderGyroDrive(2500, 1, 0);
            sleep(100);
            gyroActions.encoderGyroStrafe(2400, 14, 0, true);
            telemetry.addData(location, "<");
            telemetry.update();
        } else {
            //              Location 1
            gyroActions.encoderGyroDrive(2500, 1, 0);
            sleep(100);
            gyroActions.encoderGyroStrafe(2300, 39, 0, true);
            telemetry.addData(location, "<");
            telemetry.update();
        }
    }
    private void goToCone() {
        while (state != 6) {
            if (state == 0) {
                encoderActions.encoderSpinNoWhile(300, -90, true);
                attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                attachmentActions.liftScissor(3000, 290, true);
                state = 1;
            }
            if (state == 1 && !encoderActions.motorFrontL.isBusy()) {
                state = 2;
            }
            if (state == 2) {
                gyroActions.encoderGyroDrive(700, 47, -90);
                if (gyroActions.driveState == 0) {
                    state = 3;
                }
            }
            if (state == 3 && attachmentActions.isDone) {
                attachmentActions.closeGripper();
                startTime = System.currentTimeMillis();
                state = 4;
            }
            if (state == 4 && System.currentTimeMillis() - startTime > 650) {
                attachmentActions.liftScissor(3000, 24, false);
                state = 5;
            }
            if (state == 5 && attachmentActions.getLiftHeight() > 10) {
                state = 6;
            }
            attachmentActions.turnTableEncoders(-90, false);
        }
    }

    private void keepCallingSetTarget(double distance) {
        encoderActions.encoderDriveNoTimer(340, distance);
        while (encoderActions.motorFrontL.isBusy()) {
            encoderActions.motorFrontL.setTargetPosition(encoderActions.motorFrontL.getTargetPosition());
            encoderActions.motorFrontR.setTargetPosition(encoderActions.motorFrontR.getTargetPosition());
            encoderActions.motorBackR.setTargetPosition(encoderActions.motorBackR.getTargetPosition());
            encoderActions.motorBackL.setTargetPosition(encoderActions.motorBackL.getTargetPosition());
        }
    }

    public void liftScissor(double speed, double verticalDistance) {
        int totalTicks = (int) -verticalDistance;

        attachmentActions.scissorLift1.setTargetPosition(totalTicks);
        attachmentActions.scissorLift2.setTargetPosition(totalTicks);

        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        attachmentActions.scissorLift1.setPower(speed);
        attachmentActions.scissorLift2.setPower(speed);
    }
}
