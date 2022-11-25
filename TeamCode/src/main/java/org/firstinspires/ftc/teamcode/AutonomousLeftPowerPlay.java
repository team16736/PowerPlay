package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Blue Side Left")
public class AutonomousLeftPowerPlay extends HelperActions{

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

            attachmentActions.closeGripper();
            attachmentActions.liftScissor(1000, 1, false);

            //Move forward for space to turn table
            encoderActions.encoderDrive(speed, 2);
            sleep(500);

            //Turn table forward
            attachmentActions.turnTableEncoders(-90, 200);
            attachmentActions.tableEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //Move to position for dropping cone
            encoderActions.encoderStrafe(speed, 29, false);
            sleep(500);
            encoderActions.encoderDrive(speed, 52);
            sleep(500);

            //Drop cone
            attachmentActions.setLiftLevel(false, true, false);
            attachmentActions.turnTableEncoders(-120, 200);
            while (attachmentActions.scissorLift1.isBusy()){}
            attachmentActions.turnTableEncoders(-10, 200);
            attachmentActions.openGripper();

            //Return grabber to front
            attachmentActions.turnTableEncoders(10, 200);
            attachmentActions.liftScissor(1000, 5, false);
            attachmentActions.turnTableEncoders(120, 200);

            sleep(10000);

            //Go to pickup for new cone
            encoderActions.encoderSpin(speed, 90, true);
            sleep(500);
            encoderActions.encoderDrive(speed, 52);
            sleep(500);

            //If time is low, finish
            if(getRuntime() < 25) {
                moveToLocation(encoderActions, location);
            }

            //Pickup cone
            attachmentActions.closeGripper();
            attachmentActions.setLiftLevel(true, false, false);

            //Place cone
            placeCone(encoderActions, attachmentActions);

            //If time is low, finish
            if(getRuntime() < 25) {
                moveToLocation(encoderActions, location);
            }

            //Return grabber to front
            attachmentActions.turnTableEncoders(10, 200);
            attachmentActions.liftScissor(1000, 4, false);
            attachmentActions.turnTableEncoders(120, 200);

            //Get new cone
            encoderActions.encoderDrive(speed, 2);
            attachmentActions.closeGripper();

            //Place cone
            placeCone(encoderActions, attachmentActions);

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
            encoderActions.encoderDrive(speed, -26);
            sleep(500);
            telemetry.addData(")", "<");
            telemetry.update();
        } else {
            //              Location 3
            encoderActions.encoderDrive(speed, -52);
            sleep(500);
            telemetry.addData(")", "<");
            telemetry.update();
        }
    }
}
