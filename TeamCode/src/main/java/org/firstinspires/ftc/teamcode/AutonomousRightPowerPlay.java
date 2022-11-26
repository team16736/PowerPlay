package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Blue Side Right")
public class AutonomousRightPowerPlay extends HelperActions{

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
        driveActions.setMotorDirection_Forward();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            String location = findImageOnCone.findObject();
            sleep(10000);

            attachmentActions.closeGripper();
            sleep(500);

            attachmentActions.setLiftLevel(false, true, false);

            encoderActions.encoderStrafe(400, 21.5, true);
            sleep(500);

            encoderActions.encoderDrive(300, 41);
            sleep(500);

            attachmentActions.openGripper();
            sleep(500);

            attachmentActions.liftScissor(1000, 0, true);

            encoderActions.encoderStrafe(400, 4, true);

            encoderActions.encoderDrive(400, 11);

            moveToLocation(encoderActions, location);
        }
    }

    private void placeCone(EncoderActions encoderActions, AttachmentActions attachmentActions){
        encoderActions.encoderDrive(speed, -2);
        attachmentActions.setLiftLevel(true, false, false);
        attachmentActions.turnTableEncoders(-120, 200);
        while (attachmentActions.scissorLift1.isBusy()) {}
        attachmentActions.turnTableEncoders(-10, 200);
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
}
