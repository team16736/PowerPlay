package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name = "Autonomous Right Powerplay 3 Cone")
public class AutonomousRightPowerPlay3Cone extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private FindImageOnCone findImageOnCone = null;
    private DistanceSensorActions s1 = null;

    int state = 0;
    double startTime;
    public int coneNum = 5;
    double distanceFromCones;
    double ticksAtDistance;
    boolean distanceMemBit = false;
    int strafeSpeed;

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
            double startTime = System.currentTimeMillis();

//            attachmentActions.turnTableEncoders(0, false);
//            gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//            while (!attachmentActions.isDone || findJunctionAction.state != 0) {
//                attachmentActions.turnTableEncoders(0, false);
//                if (gyroActions.driveState != 0) {
//                    gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//                }
//            }

            //attachmentActions.turnTableEncoders(90, false); //turn turntable to the back
            //while (!attachmentActions.isDone) { //while we arent lined up with the junction,
            //    attachmentActions.turnTableEncoders(90, false); //let turntable finish turning to the back
            //}
            //sleep(10000);
            attachmentActions.closeGripper(); //Close gripper around preloaded cone
            String location = findImageOnCone.findObject(); //Detect parking spot
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            attachmentActions.liftScissor(3000, 11, false); //Lift scissor to 11 inches
//            encoderActions.encoderStrafe(400, 6, false);
            findJunctionAction.findJunction(43, 24, true, FORWARDS); //Drive to and align with pole
            attachmentActions.liftScissor(3000, 220, true); //Lower scissor slightly
            attachmentActions.openGripper(); //Release cone
            RobotLog.dd("FindJunction", "Drive to Junction 1");
            gyroActions.encoderGyroDriveStateMachine(500, 13, 0); //Drive forwards 10 inches to push signal out of the way
            double turnTableDegrees = -180;
            attachmentActions.turnTableEncoders(turnTableDegrees, false); //Turn turntable 180 degrees clockwise to point at stack
            while (gyroActions.encoderGyroDriveStateMachine(500, 14, 0)) {
                attachmentActions.turnTableEncoders(turnTableDegrees, false); //Allows it to complete drive forwards
            }
            gyroActions.encoderGyroDriveStateMachine(500, -1, 0); //Drive back an inch to ensure signal is out of the way
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
            while (gyroActions.encoderGyroDriveStateMachine(500, -1, 0)) {
                attachmentActions.turnTableEncoders(turnTableDegrees, false); //Allow previous to finish
            }
//            double previousTime = System.currentTimeMillis(); //Is this really necessary? Commented out by Wyatt 12/31/2022
//            while (System.currentTimeMillis()-previousTime < 100) { //still finishing turn, apparently 100ms
//                attachmentActions.turnTableEncoders(180, false);
//            }
            gyroActions.initEncoderGyroStrafeStateMachine(700, 19, false); //strafe right 20 inches to stack
            while (gyroActions.encoderGyroStrafeStateMachine(700, 19, 0, false)) { //while still driving:
                attachmentActions.turnTableEncoders(turnTableDegrees, false); //keep turning to 180 degrees..?
            }
            attachmentActions.closeGripper(); //Close around top cone on stack
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            RobotLog.dd("FindJunction", "Drive to Cone 2");
            attachmentActions.liftScissor(3000, 10, false); //Lift scissor to 10 inches
            coneNum--; //Subtracts cone number by one
            while (attachmentActions.getLiftHeight() < 9){} //Pause until the lift is above 9 inches to not tip over stack (!!!)
            findJunctionAction.findJunctionStateMachine(39, 26, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 37 in
            turnTableDegrees = -90;
            attachmentActions.turnTableEncoders(turnTableDegrees, false); //turn turntable to the back
            while (findJunctionAction.state != 0) { //while we arent lined up with the junction,
                attachmentActions.turnTableEncoders(turnTableDegrees, false); //let turntable finish turning to the back
                findJunctionAction.findJunctionStateMachine(37, 26, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 36 in
            }
            RobotLog.dd("FindJunction", "Drive to Junction 2");
            placeCone(attachmentActions, findJunctionAction, encoderActions);
//            if (System.currentTimeMillis() - startTime < 20000) {
//                placeCone(attachmentActions, findJunctionAction, encoderActions);
//            }
            moveToLocation(gyroActions, location);
            telemetry.addData("time", System.currentTimeMillis() - startTime);
            telemetry.update();
            RobotLog.dd("FindJunction", "Time %f", (System.currentTimeMillis() - startTime));
        }
    }

    private void placeCone(AttachmentActions attachmentActions, FindJunctionAction findJunctionAction, EncoderActions encoderActions) {
        attachmentActions.liftScissor(3000, 1500, true);
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        attachmentActions.scissorLift1.setPower(1.0);
        attachmentActions.scissorLift2.setPower(1.0);
        sleep(150);
        attachmentActions.liftScissor(3000, 1500, true);
        double strafeDistance = 32;
        if (coneNum == 3 || coneNum == 4) {strafeDistance = 30;}
        strafeSpeed = 650;
        double turnTableDegrees = -180;
        gyroActions.encoderGyroDriveStateMachine(700, -1, 0);
        while (gyroActions.encoderGyroDriveStateMachine(700, -1, 0)) {
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
        }
        attachmentActions.scissorLift1.setPower(0.0);
        attachmentActions.scissorLift2.setPower(0.0);
        gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, false);
        //Run turntable first bc cannot run both(would hit junction) and turntable is slower than lowering the lift
        while (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < 235) {
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
            gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
            getDistance(attachmentActions, encoderActions);
        }
        int grabHeight = 180;
        if (coneNum == 4) {grabHeight = 200;}
        attachmentActions.liftScissor(3000, 400, true);
        attachmentActions.openGripper();
        while (gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false)) {
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
            while (distanceMemBit == false) {
                attachmentActions.turnTableEncoders(turnTableDegrees, false);
                getDistance(attachmentActions, encoderActions);
                gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
            }
            attachmentActions.liftScissor(3000, grabHeight, true);
            telemetry.addData("is Done", true);
            telemetry.update();
        }
        while (attachmentActions.scissorLift1.isBusy() || attachmentActions.scissorLift2.isBusy()) {
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
        }
        attachmentActions.closeGripper();
        sleep(500);
        coneNum--;
        attachmentActions.liftScissor(3000, 10, false);
        while (attachmentActions.getLiftHeight() < 9){}
        int offset = 0;
        findJunctionAction.findJunctionStateMachine(40, 26, false, true, LEFT, offset, 0);
        turnTableDegrees = -90;
        attachmentActions.turnTableEncoders(turnTableDegrees, false);
        while (findJunctionAction.state != 0) {
            attachmentActions.turnTableEncoders(turnTableDegrees, false);
            findJunctionAction.findJunctionStateMachine(40, 26, false, true, LEFT, offset, 0);
        }
        RobotLog.dd("FindJunction", "Drive to Junction 3");
        distanceMemBit = false;
    }

    private void getDistance(AttachmentActions attachmentActions, EncoderActions encoderActions) {
        if (attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() + 180) < 5 && distanceMemBit == false && s1.getSensorDistance() < 10) {
//        if (Math.abs(attachmentActions.getTurntablePosition() - 180) < 10) {
            double raw = s1.getSensorDistance();
            double expSmoothed = s1.getExponentialSmoothedDistance();
            distanceFromCones = s1.getAverageDistanceAllInOne() - 5;
            telemetry.addData("avg distance", distanceFromCones);
            telemetry.addData("raw", raw);
            telemetry.addData("exp smoothed", expSmoothed);
            telemetry.update();
            gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, distanceFromCones, false);
            distanceMemBit = true;
        }
    }

    private void moveToLocation(GyroActions gyroActions, String location) {
        if (location == "Racket") {
            //            location 3
            gyroActions.encoderGyroDriveStateMachine(2500, 1, 0);
            while (gyroActions.encoderGyroDriveStateMachine(2500, 1, 0)) {}
            sleep(100);
            gyroActions.initEncoderGyroStrafeStateMachine(2000, 41, false);
            while (gyroActions.encoderGyroStrafeStateMachine(2000, 41, 0, false)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            gyroActions.encoderGyroDriveStateMachine(2500, 1, 0);
            while (gyroActions.encoderGyroDriveStateMachine(2500, 1, 0)) {}
            sleep(100);
            gyroActions.initEncoderGyroStrafeStateMachine(2000, 16, false);
            while (gyroActions.encoderGyroStrafeStateMachine(2000, 16, 0, false)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        } else {
            //              Location 1
            gyroActions.initEncoderGyroStrafeStateMachine(2000, 18, true);
            while (gyroActions.encoderGyroStrafeStateMachine(2000, 18, 0, true)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        }
        attachmentActions.turnTable.setPower(0.0);
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
