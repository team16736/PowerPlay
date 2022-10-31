package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Red Side Right")
public class AutonomousRightRed extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            double speed = 762.2;
            encoderActions.encoderDrive(speed, 15);
            encoderActions.encoderStrafe(speed, 2, false);
            placeBlock(encoderActions, attachmentActions, elementDetection(encoderActions, attachmentActions, false));

            encoderActions.encoderDriveUntilTape(3000, attachmentActions);

            encoderActions.resetEncoder();
//            encoderActions.encoderDriveUntilTape(-speed, attachmentActions);
            sleep(100);
            attachmentActions.closeGripper();
            sleep(200);
            attachmentActions.openGripper();
            sleep(200);
        }
    }
    private void placeBlock(EncoderActions encoderActions, AttachmentActions attachmentActions, int blockPlace) {
    }
}
