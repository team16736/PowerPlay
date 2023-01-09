package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Right PowerPlay")
public class AutonomousRightPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;

    private FindImageOnCone findImageOnCone = null;
    private double speed = 200;

    public void runOpMode() {
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        DistanceSensorActions baseSensor = new DistanceSensorActions(hardwareMap, 0.2, 10, ConfigConstants.BASE_RANGE);
        driveActions.setMotorDirection_Forward();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            encoderActions.encoderStrafe(400, 6, false);
            String location = findImageOnCone.findObject();

            attachmentActions.closeGripper();
            sleep(500);

            attachmentActions.setLiftLevel(false, true, false);

            encoderActions.encoderStrafe(400, 27.5, true);
            sleep(500);

            encoderActions.encoderDrive(300, 40);
            sleep(500);

            attachmentActions.setLiftLevel(true, false, false);

            attachmentActions.openGripper();
            sleep(500);

            encoderActions.encoderStrafe(400, 2, true);

            encoderActions.encoderDrive(400, 11);

//            encoderActions.encoderStrafe(400, 50, false);

            /*attachmentActions.setLiftLevel(true, false, false);
            while (attachmentActions.scissorLift1.isBusy()) {
            }

            double coneAngle = findCone(attachmentActions, baseSensor);
            attachmentActions.turnTableEncoders(coneAngle, false);
            while (!attachmentActions.isDone && opModeIsActive()) {
                attachmentActions.turnTableEncoders(coneAngle, false);
                telemetry.addData("found angle", coneAngle);
            }
            double distance = baseSensor.getAverageDistance();
            GeometryActions geometry = new GeometryActions(distance + 8.5625, coneAngle);
            encoderActions.encoderDrive(200, -geometry.getXFromDistanceAndAngle());
            attachmentActions.turnTableEncoders(0, false);
            while (!attachmentActions.isDone) {
                attachmentActions.turnTableEncoders(0, false);
            }
            encoderActions.encoderStrafe(200, - (geometry.getYFromDistanceAndAngle() - 12.5625), false);*/


            moveToLocation(encoderActions, location);
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
            //            location 1
            telemetry.addData(")", "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            encoderActions.encoderStrafe(400, 25, false);
            sleep(500);
            telemetry.addData(")", "<");
            telemetry.update();
        } else {
            //              Location 3
            encoderActions.encoderStrafe(400, 52, false);
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
