package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.FindJunctionAction;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Left")
public class AutonomousLeftPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private FindImageOnCone findImageOnCone = null;
    private GyroActions gyroActions = null;
    private double speed = 200;

    public void runOpMode() {
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        driveActions.setMotorDirection_Forward();
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        DistanceSensorActions s1 = new DistanceSensorActions(hardwareMap, 0.2, 10, ConfigConstants.BASE_RANGE);
        FindJunctionAction findJunctionAction = new FindJunctionAction(hardwareMap, telemetry, this, driveActions, attachmentActions, s1, encoderActions, gyroActions);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
//            attachmentActions.closeGripper();
            findImageOnCone.tfod.setZoom(1.5, 16.0/9.0); //set up tensorflow lite
            gyroActions.encoderGyroStrafe(700, 2, 0, false); //strafe right 2 inches
            attachmentActions.closeGripper(); //Close gripper around preloaded cone
            String location = findImageOnCone.findObject(); //Detect parking spot
            sleep(350); //Allow gripper to close - Changed from 500ms to 350ms by Wyatt 12/31/2022
            attachmentActions.extendArm(5.25); //raise grabber on minilift to 5.25 inches
            gyroActions.encoderGyroStrafe(700, 2, 0, true);
            attachmentActions.liftScissor(3000, 11, false); //Lift scissor to 11 inches
//            encoderActions.encoderStrafe(400, 6, false);
            findJunctionAction.findJunctionStateMachine(43, 20, true, false, FORWARDS, 0, 0); //drive 43 inches forwards while ramping down and searching for junction
            while (findJunctionAction.state != 0) { //while it hasnt finished, continue searching
                findJunctionAction.findJunctionStateMachine(43, 20, true, false, FORWARDS, 0, 0);
            }

            encoderActions.encoderStrafe(400, 2, true); //drive 2 inches left

            encoderActions.encoderDrive(400, 11); //drive forwards 11 inches
            moveToLocation(encoderActions, location); //park


             }
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
}
