package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Left Power Play")
public class AutonomousLeftPowerPlay extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private FindImageOnCone findImageOnCone = null;
    private double speed = 200;

    public void runOpMode() {
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        driveActions.setMotorDirection_Forward();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
//            attachmentActions.closeGripper();
            sleep(1000);
            //attachmentActions.liftScissor(1000, 8, false);
            attachmentActions.setLiftLevel(false, false, true);
            encoderActions.encoderStrafe(speed, 6, false);
//            attachmentActions.turnTableEncoders(-180, 0.00044, 0.00000016, 0.5);
//            while (!attachmentActions.isDone) {
//                attachmentActions.turnTableEncoders(-180, 0.00044, 0.00000016, 0.5);
//            }
            String location = findImageOnCone.findObject();
            //sleep(500);
            //attachmentActions.setLiftLevel(false, false, true);
            encoderActions.encoderStrafe(speed, 24, false);
            encoderActions.encoderDrive(speed, 42.5);
            attachmentActions.liftScissor(3000, 0, true);
            sleep(500);
            attachmentActions.openGripper();
            sleep(500);
            telemetry.addData("location", location);
            telemetry.update();
            moveToLocation(encoderActions, location);


             }
    }

    private void moveToLocation(EncoderActions encoderActions, String location) {
        if (location == "Cow") {
            //            location 1
            encoderActions.encoderDrive(speed,12);
            encoderActions.encoderStrafe(speed, 51, true);
            telemetry.addData("cow )", "<");
            telemetry.update();
        } else if (location == "Bus") {
            //                 location 2
            encoderActions.encoderDrive(speed, 12);
            encoderActions.encoderStrafe(speed,27, true);
            sleep(500);
            telemetry.addData("bus )", "<");
            telemetry.update();
        } else {
            //              Location 3
            sleep(500);
            telemetry.addData("racket )", "<");
            telemetry.update();
        }
        sleep(5000);
    }
}
