
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveActions.setSpeed(0.8);

        while (opModeIsActive()) {
            //TODO: add functionality for red side carousel

            /** Gamepad 1 **/
//            driveActions.drive(
//                    gamepad1.left_stick_x,      //joystick controlling strafe
//                    -gamepad1.left_stick_y,     //joystick controlling forward/backward
//                    gamepad1.right_stick_x);    //joystick controlling rotation

            if (gamepad1.x) { }
            if (gamepad1.y) { }
            if (gamepad1.a) { }
            if (gamepad1.b) { }
            attachmentActions.setLiftLevel(gamepad2.dpad_down, gamepad2.dpad_left || gamepad2.dpad_right, gamepad2.dpad_up);

            y = gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
            attachmentActions.scissorLift1.setPower(y);
            attachmentActions.scissorLift2.setPower(y);


            double armSpeed = changeSpeedArm(gamepad2.dpad_up, gamepad2.dpad_down);

            changeSpeed(driveActions, gamepad1.dpad_up || gamepad1.x, gamepad1.dpad_down || gamepad1.b, gamepad1.a, gamepad1.y);
            telemetry.addData("lift ticks", attachmentActions.scissorLift1.getCurrentPosition());
            telemetry.addData("lift position", attachmentActions.getLiftHeight());
            telemetry.addData("Table Position", attachmentActions.tableencodercount());
            telemetry.addData("Joystick Position", gamepad2.right_stick_x);
            telemetry.addData("Table Power", attachmentActions.turnTable.getPower());
            telemetry.update();
        }
        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
}
