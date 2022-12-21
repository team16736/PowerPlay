package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Right PowerPlay 2 Cone")
public class AutonomousRightPowerPlay2Cone extends HelperActions{

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
            int totalTicks = -200;

//            speed = 1000;
//            setHeightStatic(totalTicks);
//            attachmentActions.closeGripper();
//            encoderActions.encoderStrafe( 400, 1, false);
//            holdSteady(totalTicks, 500);
//            attachmentActions.setLiftLevel(false,true, false);
//            attachmentActions.turnTableEncoders(90,true);
            //  encoderActions.encoderStrafeNoWhile(400, 24, true);
            // while (!attachmentActions.isDone && encoderActions.isBusy()){
            //   attachmentActions.turnTableEncoders(90,true);
            // }


//            attachmentActions.setLiftLevel(false, true, false);
//            sleep(2000);
//            attachmentActions.liftScissor(1000, 0, true);
//            sleep(2000);
//            attachmentActions.liftScissor(500, 265, true );
//
//           // attachmentActions.liftScissor(1000, 265 , true );
//            sleep(10000);
//            encoderActions.encoderStrafe(400, 6, false);
//            String location = findImageOnCone.findObject();
//
//            attachmentActions.closeGripper();
//            sleep(500);
//
//            attachmentActions.setLiftLevel(false, true, false);
//
//            encoderActions.encoderStrafe(400, 27.5, true);
//            sleep(500);
//
//            encoderActions.encoderDrive(300, 42);
//            sleep(500);
//
//            attachmentActions.liftScissor(1000, 0, true);
//
//            attachmentActions.openGripper();
//            sleep(500);
//
//            encoderActions.encoderStrafe(400, 2, true);
//
//            encoderActions.encoderDrive(400, 13.5);
//
//            // strafe to cone to pick up
//            //encoderActions.encoderDrive(400,2);
//            attachmentActions.liftScissor(500, 265, true );
//            encoderActions.encoderStrafe(400, 50.50, false);
//            sleep(250);
//            attachmentActions.closeGripper();


//            encoderActions.encoderStrafe(400, 52, false);
//            findCone(attachmentActions);


            //moveToLocation(encoderActions, location);
        }
    }

    private void setHeightStatic(int totalTicks) {
        attachmentActions.scissorLift1.setTargetPosition(totalTicks);
        attachmentActions.scissorLift2.setTargetPosition(totalTicks);

        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            DcMotor.RunMode.
        attachmentActions.scissorLift1.setVelocity(speed);
        attachmentActions.scissorLift2.setVelocity(speed);
        while (attachmentActions.scissorLift1.isBusy()) {
            telemetry.addData("scissor lift height reached", !attachmentActions.scissorLift1.isBusy());
            telemetry.update();
        }
    }

    private void holdSteady(int totalTicks, double timeToWait) {
        int tolerance = 5;
        double baseTime = System.currentTimeMillis();
        double currentTime = System.currentTimeMillis();
        double timeDiff = currentTime - baseTime;
        while (timeDiff < timeToWait) {
            currentTime = System.currentTimeMillis();
            timeDiff = currentTime - baseTime;

            int positionDiff = totalTicks - attachmentActions.scissorLift1.getCurrentPosition();

            if (Math.abs(positionDiff) > tolerance){
                attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                attachmentActions.scissorLift1.setVelocity(speed);
                attachmentActions.scissorLift2.setVelocity(speed);
                while (attachmentActions.scissorLift1.isBusy()) {
                    telemetry.addData("scissor lift height adjustment",attachmentActions.scissorLift1.getCurrentPosition());
                    telemetry.update();
                }

            }
            telemetry.addData("scissor stable",attachmentActions.scissorLift1.getCurrentPosition());
            telemetry.update();
        }
    }

    private void placeCone(EncoderActions encoderActions, AttachmentActions attachmentActions){
        encoderActions.encoderDrive(speed, -2);
        attachmentActions.setLiftLevel(true, false, false);
        attachmentActions.turnTableEncoders(-120, true);
        while (attachmentActions.scissorLift1.isBusy()) {}
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
    private void findCone(AttachmentActions attachmentActions) {
        double angle = 45;
        double turnTableSpeed = 0.2;
        double minDistance = 1000;
        double minDistanceAngle = 0;
//        attachmentActions.turnTableEncoders(angle, turnTableSpeed);
        while (!attachmentActions.isDone && opModeIsActive()) {
            attachmentActions.turnTableEncoders(angle, false);
//            if (attachmentActions.getJunctionDistance() < minDistance) {
//                minDistance = attachmentActions.getJunctionDistance();
//                minDistanceAngle = attachmentActions.getTurntablePosition();
//            }
        }

    }
}