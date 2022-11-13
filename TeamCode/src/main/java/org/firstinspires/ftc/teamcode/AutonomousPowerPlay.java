

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Power Play")

public class AutonomousPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private double speed = 200;

    public void runOpMode() {
        EncoderActions encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        FindImageOnCone findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        driveActions.setMotorDirection_Forward();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            String location = findImageOnCone.findObject();
            moveToLocation(encoderActions, location);


        }
    }

    private void moveToLocation(EncoderActions encoderActions, String location) {
        if (location == "Cow") {
            //            location 1
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 23, true);
            encoderActions.encoderDrive(speed, 26);
            telemetry.addData(">", "We Can Drive!");
            telemetry.update();
        } else if (location == "Bus") {
            //sleep(10000);
            //                 location 2
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 4.5, false);
            encoderActions.encoderDrive(speed, 26);
        } else {
            //              Location 3
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 29, false);
            encoderActions.encoderDrive(speed, 26);
                    }
    }
}