package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "PROGGERS Right Powerplay 3 Cone")
// james suggested "PROGGERS" not me, -wyatt
public class Right3ConeInProg extends HelperActions{
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
//        findImageOnCone.tfod.setClippingMargins(0, 0, 0, 0);

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

            attachmentActions.closeGripper(); //Close gripper around preloaded cone
            String location = findImageOnCone.findObject(); //Detect parking spot
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            attachmentActions.extendArm(2); //raise mini lift 2 inches from lowest pos
            attachmentActions.liftScissor(3000, 10, false); //Lift scissor to 10 inches
//            encoderActions.encoderStrafe(400, 6, false);
            findJunctionAction.findJunction(43, 24, true, FORWARDS); //Drive to and align with pole
//            sleep(200000);
            attachmentActions.liftScissor(3000, 260, true); //Lower scissor slightly
            attachmentActions.openGripper(); //Release cone
            RobotLog.dd("FindJunction", "Drive to Junction 1");
            gyroActions.encoderGyroDriveStateMachine(500, 11, 0); //Drive forwards 11 inches
            attachmentActions.turnTableEncoders(-180, false); //Turn turntable 180 degrees counterclockwise to point at stack
            while (gyroActions.encoderGyroDriveStateMachine(500, 11, 0)) {
                attachmentActions.turnTableEncoders(-180, false); //Allows it to complete drive forwards
            }
            attachmentActions.turnTableEncoders(-180, false);
            gyroActions.initEncoderGyroStrafeStateMachine(700, 17.5, false); //strafe right 20 inches to stack
            attachmentActions.extendArm(0); //added by Wyatt 1/18/2023
            while (gyroActions.encoderGyroStrafeStateMachine(700, 17.5, 0, false)) { //while still driving:
                attachmentActions.turnTableEncoders(-180, false); //keep turning to 180 degrees..?
            }
            attachmentActions.closeGripper(); //Close around top cone on stack
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            RobotLog.dd("FindJunction", "Drive to Cone 2");
            attachmentActions.liftScissor(3000, 10, false); //Lift scissor to 10 inches
            coneNum--; //Subtracts cone number by one
            while (attachmentActions.getLiftHeight() < 9){} //Pause until the lift is above 9 inches to not tip over stack (!!!)
            findJunctionAction.findJunctionStateMachine(37, 26, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 37 in
            attachmentActions.turnTableEncoders(-90, false); //turn turntable to the back
            while (findJunctionAction.state != 0) { //while we arent lined up with the junction,
                attachmentActions.turnTableEncoders(-90, false); //let turntable finish turning to the back
                findJunctionAction.findJunctionStateMachine(37, 26, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 36 in
            }
            RobotLog.dd("FindJunction", "Drive to Junction 2");
            /*
            placeCone is a singular method that drives to the stack, picks up off the stack, searches for the junction.
             */
            placeCone(attachmentActions, findJunctionAction, encoderActions); //call method to find cone #3
            if (System.currentTimeMillis() - startTime < 20000) {
                placeCone(attachmentActions, findJunctionAction, encoderActions); //if elapsed time < 20 seconds, go for cone #4
            }
            moveToLocation(gyroActions, location);
            telemetry.addData("time", System.currentTimeMillis() - prevTime);
            telemetry.update();
            RobotLog.dd("FindJunction", "Time %f", (System.currentTimeMillis() - prevTime));
        }
    }

    private void placeCone(AttachmentActions attachmentActions, FindJunctionAction findJunctionAction, EncoderActions encoderActions) {
        attachmentActions.liftScissor(3000, 1500, true);   //set scissor height to 1500 ticks. intermediate pos, 1500 is barely not enough
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   //why?
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   //why?
        attachmentActions.scissorLift1.setPower(1.0);   //why?
        attachmentActions.scissorLift2.setPower(1.0);   //why?
        sleep(200);   //why?
        attachmentActions.liftScissor(3000, 1560, true);   //set scissor height to 1500 ticks. intermediate pos, 1500 is barely not enough
        double strafeDistance = 32; //sets strafe dist to 32in
        if (coneNum == 3 || coneNum == 4) {strafeDistance = 30;} //if we're on the 3rd or 4th cone on the stack, set strafe dist to 30in
        strafeSpeed = 650; //set strafe speed to 650 - normally 700. why 650? can we go a little faster?
        gyroActions.encoderGyroDriveStateMachine(700, -1, 0); //drive backwards an inch to align with stack
        while (gyroActions.encoderGyroDriveStateMachine(700, -1, 0)) {
            attachmentActions.turnTableEncoders(-180, false); //turn turntable
        }
        //while doing the above, the scissor lift runs into the junction. possibly raise scissor from 1500 to 1550? 1600?
        attachmentActions.scissorLift1.setPower(0.0);   //idle lift
        attachmentActions.scissorLift2.setPower(0.0);   //idle lift
        gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, false); //strafe left 30" if on cone 3 or 4, 32" if on top cone. strafe at 700tpr
        //Run turntable first bc cannot run both(would hit junction) and turntable is slower than lowering the lift
        while (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < 235) { //where does 235 come from?
            attachmentActions.turnTableEncoders(-180, false); //allow completion of turn
            gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false); //continue the strafe
            getDistance(attachmentActions, encoderActions);
        }
        int grabHeight = 220;
//        if (coneNum == 4) {grabHeight = 210;}
        attachmentActions.liftScissor(3000, 400, true);
        attachmentActions.openGripper();
        while (gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false)) {
            attachmentActions.turnTableEncoders(-180, false);
            while (distanceMemBit == false) {
                attachmentActions.turnTableEncoders(-180, 0.00044, 0.00000024, 0.5);
                getDistance(attachmentActions, encoderActions);
                gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
            }
            attachmentActions.liftScissor(3000, grabHeight, true);
            telemetry.addData("is Done", true);
            telemetry.update();
        }
        while (attachmentActions.scissorLift1.isBusy() || attachmentActions.scissorLift2.isBusy()) {
            attachmentActions.turnTableEncoders(-180, false);
        }
        attachmentActions.closeGripper();
        sleep(500);
        coneNum--;
        attachmentActions.liftScissor(3000, 10, false);
        while (attachmentActions.getLiftHeight() < 9){}
        int offset = 0;
        findJunctionAction.findJunctionStateMachine(40, 26, false, true, LEFT, offset, 0);
        attachmentActions.turnTableEncoders(-90, false);
        while (findJunctionAction.state != 0) {
            attachmentActions.turnTableEncoders(-90, false);
            findJunctionAction.findJunctionStateMachine(40, 26, false, true, LEFT, offset, 0);
        }
        RobotLog.dd("FindJunction", "Drive to Junction 3");
        distanceMemBit = false;
    }

    private void getDistance(AttachmentActions attachmentActions, EncoderActions encoderActions) {
        if (attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() + 180) < 5 && distanceMemBit == false && s1.getSensorDistance() < 10) {
//        if (Math.abs(attachmentActions.getTurntablePosition() - 180) < 10) {
            distanceFromCones = s1.getAverageDistanceAllInOne(true) - 6.5;
            telemetry.addData("avg distance", distanceFromCones);
            telemetry.update();
            gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, distanceFromCones, false);
            distanceMemBit = true;
        } else if (s1.getSensorDistance() > 10 && gyroActions.strafeState == 0) {
            distanceFromCones = s1.getAverageDistanceAllInOne(true) - 11;
            gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, distanceFromCones, false);
            RobotLog.dd("FindJunction", ":/");
        }
    }

    private void moveToLocation(GyroActions gyroActions, String location) {
        if (location == "Racket") {
            //            location 3
            gyroActions.initEncoderGyroStrafeStateMachine(1500, 41, false);
            while (gyroActions.encoderGyroStrafeStateMachine(1500, 41, 0, false)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            gyroActions.initEncoderGyroStrafeStateMachine(2000, 15, false);
            while (gyroActions.encoderGyroStrafeStateMachine(2000, 15, 0, false)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        } else {
            //              Location 1
            gyroActions.initEncoderGyroStrafeStateMachine(1500, 16, true);
            while (gyroActions.encoderGyroStrafeStateMachine(1500, 16, 0, true)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        }
        attachmentActions.turnTable.setPower(0.0);
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
