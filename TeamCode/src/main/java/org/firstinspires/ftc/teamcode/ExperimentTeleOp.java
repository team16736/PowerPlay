
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;

@TeleOp(name = "Experiment Tele Op", group = "Linear Opmode")

public class ExperimentTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    boolean memoryBit;
    boolean memBitArmSpin;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        DistanceSensorActions s1 = new DistanceSensorActions(hardwareMap, 0.2, 10, ConfigConstants.GRABBER_RANGE);
        DistanceSensorActions s2 = new DistanceSensorActions(hardwareMap, 0.2, 10, ConfigConstants.BASE_RANGE);

        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        double carouselPower = 0.4;
        int targetArmSpin = 0;
        int speeding = 0;
        double speed = 0.8;
        double y; //Create new double for the speed.
        int currentPos; //Create an integer for the current position (IMPORTANT THAT ITS AN INTEGER, WILL NOT WORK OTHERWISE)
        final float[] hsvValues = new float[3];

        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveActions.setSpeed(0.8);

        attachmentActions.setLiftLevel(true, false, false);

        while (opModeIsActive()) {
            //TODO: add functionality for red side carousel

            /** Gamepad 1 **/
//            driveActions.drive(
//                    gamepad1.left_stick_x,      //joystick controlling strafe
//                    -gamepad1.left_stick_y,     //joystick controlling forward/backward
//                    gamepad1.right_stick_x);    //joystick controlling rotation

//            if (gamepad1.x) { }
//            if (gamepad1.y) { }
//            if (gamepad1.a) { }
//            if (gamepad1.b) { }
//            attachmentActions.setLiftLevel(gamepad2.dpad_down, gamepad2.dpad_left || gamepad2.dpad_right, gamepad2.dpad_up);
//
//            y = gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
//            attachmentActions.scissorLift1.setPower(y);
//            attachmentActions.scissorLift2.setPower(y);
//
//
//            double armSpeed = changeSpeedArm(gamepad2.dpad_up, gamepad2.dpad_down);
//
//            changeSpeed(driveActions, gamepad1.dpad_up || gamepad1.x, gamepad1.dpad_down || gamepad1.b, gamepad1.a, gamepad1.y);
            telemetry.addData("s1 raw", s1.getSensorDistance());
            telemetry.addData("s1 averaged", s1.getAverageDistance());
            telemetry.addData("s1 expo smoothed", s1.getExponentialSmoothedDistance());
            telemetry.addData("s2 raw", s2.getSensorDistance());
            telemetry.addData("s2 averaged", s2.getAverageDistance());
            telemetry.addData("s2 expo smoothed", s2.getExponentialSmoothedDistance());
            telemetry.update();
        }
        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
}
