

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Left Side Red Power Play")

public class AutonomousLeftSideRedPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private double speed = 200;

    public void runOpMode() {
        driveActions = new DriveActions(telemetry, hardwareMap);

        EncoderActions encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        //FindImageOnCone findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        //attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

        int location = 1;
            moveToLocation(encoderActions, location);


        }
    }

    private void moveToLocation(EncoderActions encoderActions, int location) {
        if (location == 1) {
            //            location 1
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 21, true);
            encoderActions.encoderDrive(speed, 26);
            telemetry.addData(">", "We Can Drive!");
            telemetry.update();
        } else if (location == 2) {
            //sleep(10000);
            //                 location 2
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 3, false);
            encoderActions.encoderDrive(speed, 26);
        } else {
            //              Location 3
            encoderActions.encoderDrive(speed, 2);
            encoderActions.encoderStrafe(speed, 28, false);
            encoderActions.encoderDrive(speed, 26);
        }
    }
}
