package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindJunctionAction;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Right")
public class AutonomousRightPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private GeometryActions geometry = null;

    private FindImageOnCone findImageOnCone = null;
    private double speed = 200;

    public void runOpMode() {
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        DistanceSensorActions s1 = new DistanceSensorActions(hardwareMap, 0.2, 10, ConfigConstants.BASE_RANGE);
        geometry = new GeometryActions();
        FindJunctionAction findJunctionAction = new FindJunctionAction(hardwareMap, telemetry, this, driveActions, attachmentActions, s1, encoderActions, gyroActions, geometry);
        driveActions.setMotorDirection_Forward();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            findImageOnCone.tfod.setZoom(1.5, 16.0/9.0); //init tfod
            attachmentActions.closeGripper(); //Close gripper around preloaded cone
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            gyroActions.encoderGyroStrafe(700, 2, 0, true); //strafe left 2 inches
            String location = findImageOnCone.findObject(); //Detect parking spot
            attachmentActions.extendArm(5.25); //set minilift height to 5.25 inches
            gyroActions.encoderGyroStrafe(700, 2, 0, false); //strafe right 2 inches
            attachmentActions.liftScissor(3000, 11, false); //Lift scissor to 11 inches
            findJunctionAction.findJunction(43, 20, true, FORWARDS); //drive 43 inches forwards ramping down speed searching for junction
            attachmentActions.liftScissor(3000, 17, false); //lift scissor to 17 inches
            encoderActions.encoderStrafe(400, 2, false); //strafe right 2 inches
            encoderActions.encoderDrive(400, 11); //drive forwards 11 inches
            moveToLocation(encoderActions, location); //move to parking spot
        }
    }

    private void placeCone(EncoderActions encoderActions, AttachmentActions attachmentActions) {
        encoderActions.encoderDrive(speed, -2);
        attachmentActions.setLiftLevel(true, false, false);
        attachmentActions.turnTableEncoders(-120, true);
        while (attachmentActions.scissorLift1.isBusy()) {
        }
        attachmentActions.turnTableEncoders(-10, true);
        attachmentActions.openGripper();
    }

    private void moveToLocation(EncoderActions encoderActions, String location) {
        if (location == "Cow") {
            encoderActions.encoderStrafe(700, 26, true);
            //            location 1
            telemetry.addData(")", "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            sleep(500);
            telemetry.addData(")", "<");
            telemetry.update();
        } else {
            //              Location 3
            encoderActions.encoderStrafe(700, 26, false);
            sleep(500);
            telemetry.addData(")", "<");
            telemetry.update();
        }
    }

    private double findCone(AttachmentActions attachmentActions, DistanceSensorActions baseSensor) {
        double angle = 45;
        double minDistance = baseSensor.getSensorDistance();
        double minDistanceAngle = 0;
        attachmentActions.turnTable.setPower(0.2);
        while (attachmentActions.getTurntablePosition() < angle && opModeIsActive()/* && Math.abs(baseSensor.getSensorDistance() - minDistance) < 2*/) {
            if (baseSensor.getSensorDistance() < minDistance) {
                minDistance = baseSensor.getSensorDistance();
                minDistanceAngle = attachmentActions.getTurntablePosition();
            }
            telemetry.addData("minDistance", minDistance);
            telemetry.addData("minDistanceAngle", minDistanceAngle);
            telemetry.addData("distance", baseSensor.getSensorDistance());
        }
        minDistance = baseSensor.getSensorDistance();
        attachmentActions.turnTable.setPower(-0.2);
        while (attachmentActions.getTurntablePosition() > -angle && opModeIsActive()/* && Math.abs(baseSensor.getSensorDistance() - minDistance) < 2*/) {
            if (baseSensor.getSensorDistance() < minDistance) {
                minDistance = baseSensor.getSensorDistance();
                minDistanceAngle = attachmentActions.getTurntablePosition();
            }
            telemetry.addData("minDistance1", minDistance);
            telemetry.addData("minDistanceAngle1", minDistanceAngle);
            telemetry.addData("distance", baseSensor.getSensorDistance());
        }
        return minDistanceAngle;
    }
}
