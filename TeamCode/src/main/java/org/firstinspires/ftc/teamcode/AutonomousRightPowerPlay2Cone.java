package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Right Powerplay 2 Cone")
@Disabled
public class AutonomousRightPowerPlay2Cone extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private FindImageOnCone findImageOnCone = null;
    private DistanceSensorActions s1 = null;
    private GeometryActions geometry = null;

    int state = 0;
    double startTime;

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

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            double prevTime = System.currentTimeMillis();

            attachmentActions.closeGripper();
            sleep(500);
            attachmentActions.liftScissor(3000, 10, false);
            encoderActions.encoderStrafe(400, 6, false);
            String location = findImageOnCone.findObject();
            encoderActions.encoderStrafe(400, 27, true);
            sleep(400);
            findJunctionAction.findJunction(47, 24, true, FORWARDS);
            encoderActions.encoderStrafe(300, 3, true);
            gyroActions.encoderGyroDrive(300, 12, 0);
            goToCone();
            attachmentActions.turnTableEncoders(0, false);
            findJunctionAction.findJunctionStateMachine(35, 26, false, true, BACKWARDS);
            while (!attachmentActions.isDone || findJunctionAction.state != 0) {
                attachmentActions.turnTableEncoders(0, false);
                if (findJunctionAction.state != 0) {
                    findJunctionAction.findJunctionStateMachine(35, 26, false, true, BACKWARDS);
                }
            }
            encoderActions.encoderStrafe(400, 2, true);
            moveToLocation(gyroActions, location);
            telemetry.addData("time", System.currentTimeMillis() - prevTime);
            telemetry.update();
        }
    }
    private void moveToLocation(GyroActions gyroActions, String location) {
        if (location == "Cow") {
            //            location 1
            gyroActions.encoderGyroDrive(700, -8, -90);
            telemetry.addData(location, "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            gyroActions.encoderGyroDrive(700, 8, -90);
            sleep(500);
            telemetry.addData(location, "<");
            telemetry.update();
        } else {
            //              Location 3
            gyroActions.encoderGyroDrive(700, 32, -90);
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
                gyroActions.encoderGyroDrive(700, 48, -90);
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
}
