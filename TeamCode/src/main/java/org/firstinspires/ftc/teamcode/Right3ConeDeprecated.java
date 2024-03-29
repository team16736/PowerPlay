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
import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit
@Disabled
@Autonomous(name = "DEPRECATED Right Powerplay 3 Cone")
public class Right3ConeDeprecated extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private FindImageOnCone findImageOnCone = null;
    private DistanceSensorActions s1 = null;
    private GeometryActions geometry = null;

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
        geometry = new GeometryActions();
        driveActions.setMotorDirection_Forward();
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FindJunctionAction findJunctionAction = new FindJunctionAction(hardwareMap, telemetry, this, driveActions, attachmentActions, s1, encoderActions, gyroActions, geometry);
        findImageOnCone.tfod.setZoom(1.75, 16.0/9.0);

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
            gyroActions.encoderGyroStrafe(500, 3, 0, true);
            String location = findImageOnCone.findObject(); //Detect parking spot
            attachmentActions.extendArm(4); //Lift up the grabber to avoid the ground
            attachmentActions.liftScissor(3000, 11, false); //Lift scissor to 11 inches
//            encoderActions.encoderStrafe(400, 6, false);
            findJunctionAction.findJunction(43, 21, true, FORWARDS); //Drive to and align with pole
            attachmentActions.liftScissor(3000, 400, true); //Set lift position to enough so that it won't interfere with sensing the cone
            attachmentActions.extendArm(5.25); //Raise the grabber high enough to pick up the first cone
            attachmentActions.openGripper(); //Release cone
            RobotLog.dd("FindJunction", "Drive to Junction 1");
            gyroActions.encoderGyroDriveStateMachine(500, 13, 0); //Drive forwards 10 inches to push signal out of the way
            double turnTableDegrees = -180;
            attachmentActions.turnTableEncoders(turnTableDegrees); //Turn turntable 180 degrees clockwise to point at stack
            while (gyroActions.encoderGyroDriveStateMachine(500, 13, 0)) {
                attachmentActions.turnTableEncoders(turnTableDegrees); //Allows it to complete drive forwards
            }
            gyroActions.encoderGyroDriveStateMachine(500, -1, 0); //Drive back an inch to ensure signal is out of the way
            attachmentActions.turnTableEncoders(turnTableDegrees);
            while (gyroActions.encoderGyroDriveStateMachine(500, -1, 0)) {
                attachmentActions.turnTableEncoders(turnTableDegrees); //Allow previous to finish
            }
//            double previousTime = System.currentTimeMillis(); //Is this really necessary? Commented out by Wyatt 12/31/2022
//            while (System.currentTimeMillis()-previousTime < 100) { //still finishing turn, apparently 100ms
//                attachmentActions.turnTableEncoders(180, false);
//            }
            gyroActions.initEncoderGyroStrafeStateMachine(700, 14.5, false); //strafe right 14.5 inches to stack
            RobotLog.dd("FindJunction", "strafeState %d, Target Pos %d", gyroActions.strafeState, encoderActions.motorFrontL.getTargetPosition());
            while (Math.abs(encoderActions.motorFrontL.getTargetPosition() - encoderActions.motorFrontL.getCurrentPosition()) < 100) {
                gyroActions.encoderGyroStrafeStateMachine(700, 14.5, 0, false);
            }
            while (!(attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() - attachmentActions.getTurntableGoal()) < 2)) {
                attachmentActions.turnTableEncoders(turnTableDegrees);
                gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, 14.5, 0, false);
                telemetry.addData("is Done", true);
                telemetry.update();
            }
            RobotLog.dd("FindJunction", "Distance Gone: %f", (double) (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) / 31.0));
            encoderActions.resetEncoder();
            driveToCone(gyroActions, s1, encoderActions, attachmentActions, 12.5, RIGHT); //Using the distance sensor to see the cones, it drives to them
            while (driveToCone(gyroActions, s1, encoderActions, attachmentActions, 12.5, RIGHT)) {
                attachmentActions.turnTableEncoders(turnTableDegrees);
            }
//            double coneDistanceFinal = s1.getSensorDistance() - 12;
//            RobotLog.dd("FindJunction", "Final Distance: %f", coneDistanceFinal);
//            gyroActions.initEncoderGyroStrafeStateMachine(100, coneDistanceFinal, false);
//            while (gyroActions.encoderGyroStrafeStateMachine(100, coneDistanceFinal, 0, false)) {
//                attachmentActions.turnTableEncoders(turnTableDegrees, false);
//            }
            attachmentActions.liftScissor(3000, -1.5, false); //Slam down lift to get the grabber in the right place
            distanceMemBit = false;
            attachmentActions.closeGripper(); //Close around top cone on stack
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            RobotLog.dd("FindJunction", "Drive to Cone 2");
            attachmentActions.liftScissor(3000, 10,false); //Lift scissor to 10 inches
            coneNum--; //Subtracts cone number by one
            while (attachmentActions.scissorLift1.getCurrentPosition() > -250){
                gyroActions.setVelocityStraight(50, 0, LEFT);
            } //Slow until the lift is above 9 inches to not tip over stack (!!!)
            findJunctionAction.findJunctionStateMachine(37, 22, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 37 in
            turnTableDegrees = -90;
            attachmentActions.turnTableEncoders(turnTableDegrees); //turn turntable to the back
            while (findJunctionAction.state != 0) { //while we arent lined up with the junction,
                attachmentActions.turnTableEncoders(turnTableDegrees); //let turntable finish turning to the back
                findJunctionAction.findJunctionStateMachine(37, 22, false, true, LEFT); //Was at 40 in for dist, changed by Wyatt on 12/31/22 to 36 in
            }
            RobotLog.dd("FindJunction", "Drive to Junction 2");
            placeCone(attachmentActions, findJunctionAction, encoderActions);
//            if (System.currentTimeMillis() - startTime < 20000) {
//                placeCone(attachmentActions, findJunctionAction, encoderActions);
//            }
            attachmentActions.extendArm(0);
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
        attachmentActions.scissorLift2.setPower(1.0); //Slam the lift downwards
        sleep(150);
        attachmentActions.liftScissor(3000, 1500, true); //Bring the lift back up to above the junction
        double strafeDistance = 34;
        if (coneNum == 3 || coneNum == 4) {strafeDistance = 30;}
        strafeSpeed = 650;
        double turnTableDegrees = -180;
        gyroActions.encoderGyroDriveStateMachine(700, -1, 0); //To align with the stack
        while (gyroActions.encoderGyroDriveStateMachine(700, -1, 0)) {
            attachmentActions.turnTableEncoders(turnTableDegrees);
        }
        attachmentActions.scissorLift1.setPower(0.0);
        attachmentActions.scissorLift2.setPower(0.0);
        gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, false);
        //Run turntable first bc cannot run both(would hit junction) and turntable is slower than lowering the lift
        while (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < 235) {
            attachmentActions.turnTableEncoders(turnTableDegrees);
            gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
            getDistance(attachmentActions, encoderActions);
        }
        attachmentActions.liftScissor(3000, 400, true); //Set lift where it isn't in the way
        attachmentActions.openGripper();
        while (!(attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() - attachmentActions.getTurntableGoal()) < 4)) { //Drive to cone, essentially the same as before
            attachmentActions.turnTableEncoders(turnTableDegrees);
//            while (distanceMemBit == false) {
//                attachmentActions.turnTableEncoders(turnTableDegrees, false);
//                getDistance(attachmentActions, encoderActions);
//                gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
//            }
            gyroActions.encoderGyroStrafeStateMachine(strafeSpeed, strafeDistance, 0, false);
            telemetry.addData("is Done", true);
            telemetry.update();
        }
        RobotLog.dd("FindJunction", "Distance Gone: %f", (double) (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) / 31.0));
        encoderActions.resetEncoder();
        double setSpeed = s1.driveToObject(7.5, 700, 400);
        gyroActions.setVelocityStraight(setSpeed, 0, RIGHT);
        while (setSpeed != 0) {
            if (attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() - attachmentActions.getTurntableGoal()) < 2) {
                setSpeed = s1.driveToObject(7.5, 700, 400);
                gyroActions.setVelocityStraight(setSpeed, 0, RIGHT);
            } else {
                gyroActions.setVelocityStraight(100, 0, RIGHT);
            }
            attachmentActions.turnTableEncoders(turnTableDegrees);
        }
        double coneDistanceFinal = s1.getSensorDistance() - 6.5;
        RobotLog.dd("FindJunction", "Final Distance: %f", coneDistanceFinal);
        gyroActions.initEncoderGyroStrafeStateMachine(100, coneDistanceFinal, false);
        while (gyroActions.encoderGyroStrafeStateMachine(100, coneDistanceFinal, 0, false)) {
            attachmentActions.turnTableEncoders(turnTableDegrees);
        }
        attachmentActions.liftScissor(3000, -1.5, false); //Slam lift down to get grabber in the right position
        distanceMemBit = false;
        while (attachmentActions.scissorLift1.getCurrentPosition() < -60) {
            attachmentActions.turnTableEncoders(turnTableDegrees);
        }
        attachmentActions.closeGripper(); //Grab cone
        sleep(500);
        coneNum--; //Decrease cone number
        attachmentActions.liftScissor(3000, 10, false); //Lift cone off of stack
        while (attachmentActions.scissorLift1.getCurrentPosition() > -250){
            gyroActions.setVelocityStraight(50, 0, LEFT);
        }
        int offset = -1;
        findJunctionAction.findJunctionStateMachine(40, 22, false, true, LEFT, offset, 0); //Go to the junction
        turnTableDegrees = -90;
        attachmentActions.turnTableEncoders(turnTableDegrees); //Put the turntable behind us
        while (findJunctionAction.state != 0) {
            attachmentActions.turnTableEncoders(turnTableDegrees);
            findJunctionAction.findJunctionStateMachine(40, 22, false, true, LEFT, offset, 0);
        }
        if (coneNum == 3) {
            gyroActions.encoderGyroDriveStateMachine(1000, 1, 0);
            while (gyroActions.encoderGyroDriveStateMachine(1000, 1, 0)) {}
        }
        RobotLog.dd("FindJunction", "Drive to Junction 3");
        distanceMemBit = false;
    }

    private void getDistance(AttachmentActions attachmentActions, EncoderActions encoderActions) {
        if (attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() + 180) < 4 && distanceMemBit == false && s1.getSensorDistance() < 10) {
//        if (Math.abs(attachmentActions.getTurntablePosition() - 180) < 10) {
            distanceFromCones = s1.getAverageDistanceAllInOne(true) - 7.5;
            telemetry.update();
            gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, distanceFromCones, false);
            distanceMemBit = true;
        } else if (s1.getSensorDistance() > 10 && gyroActions.strafeState == 0) {
            distanceFromCones = s1.getAverageDistanceAllInOne(true) - 9;
            gyroActions.initEncoderGyroStrafeStateMachine(strafeSpeed, distanceFromCones, true);
            distanceMemBit = true;
            RobotLog.dd("FindJunction", ":/");
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
            gyroActions.initEncoderGyroStrafeStateMachine(2000, 15, false);
            while (gyroActions.encoderGyroStrafeStateMachine(2000, 15, 0, false)) {}
            telemetry.addData(location, "<");
            telemetry.update();
        } else {
            //              Location 1
            gyroActions.initEncoderGyroStrafeStateMachine(1500, 18, true);
            while (gyroActions.encoderGyroStrafeStateMachine(1500, 18, 0, true)) {}
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
            attachmentActions.turnTableEncoders(-90);
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
