
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

@TeleOp(name = "Tele Op", group = "Linear Opmode")
public class MainTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);

        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        int currentTicks = 0;
        double y = 0;
        double turnTableRotation = 0;
        double encoderAdjustment;
        boolean memBitLift = true;
        int gravityThresholdLift = -350;
        boolean spinLeft;
        boolean placeBit;
        double extenderInput;
        double prevTime = 0;
        double extendLength = 1.0;
        int extendTime = 1200; //Time to fully extend the extender in milliseconds. Needs to be changed

        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachmentActions.scissorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        attachmentActions.scissorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            /** Gamepad 1 **/
            driveActions.drive(
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)),      //joystick controlling strafe
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)),     //joystick controlling forward/backward
                    (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)));    //joystick controlling rotation
            telemetry.addData("Left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick x", gamepad1.right_stick_x);
            telemetry.update();

//            attachmentActions.setConeLevel(gamepad2.dpad_down, gamepad2.dpad_left || gamepad2.dpad_right, gamepad2.dpad_up);
            attachmentActions.armPresets(gamepad2);

            if(Math.abs(gamepad2.left_stick_y) > 0.01) {
                attachmentActions.liftWithoutEncoders();
                y = gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
                attachmentActions.scissorLift1.setPower(y);
                attachmentActions.scissorLift2.setPower(y);
                memBitLift = false;
            } else if (!attachmentActions.scissorLift1.isBusy() && attachmentActions.scissorLift1.getCurrentPosition() < gravityThresholdLift) {
                attachmentActions.scissorLift1.setPower(0);
                attachmentActions.scissorLift2.setPower(0);
            }
//            if (attachmentActions.scissorLift1.getCurrentPosition() > gravityThresholdLift) {
//                attachmentActions.stayWhereSet(gamepad2.left_stick_y);
//            }

            if((Math.abs(gamepad2.left_stick_y) < 0.01) && (attachmentActions.scissorLift1.getCurrentPosition() > gravityThresholdLift) && !memBitLift){
                attachmentActions.liftScissor(3000, -attachmentActions.scissorLift1.getCurrentPosition(), true);
                memBitLift = true;
            }

            telemetry.addData("Joystick", gamepad2.right_stick_y);

//            extenderInput = gamepad2.right_trigger - gamepad2.left_trigger;
//            extendLength = Range.clip(extendLength - ((extenderInput * (System.currentTimeMillis() - prevTime) / extendTime)), 0.0, 1.0);
//            prevTime = System.currentTimeMillis();
//            attachmentActions.extender.setPosition(extendLength);

            if (gamepad2.x) {
                attachmentActions.closeGripper();
            }
            if (gamepad2.y) {
                attachmentActions.openGripper();
            }

            dudeYouShouldChill(driveActions, attachmentActions);

            changeSpeed(driveActions, gamepad1.dpad_up, gamepad1.dpad_down, false, false);

            attachmentActions.turnTable.setPower(getAdjustedTurntablePower());

            //Need to add an interrupt
            /*if (!isPlacingCone) {
                spinLeft = false;
                placeBit = false;
            }
            if (gamepad2.left_bumper) {
                spinLeft = true;
                placeBit = true;
            }
            if (gamepad2.right_bumper) {
                placeBit = true;
            }*/

//
            telemetry.addData("Current Position ", attachmentActions.scissorLift1.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
    private double getAdjustedTurntablePower() {
        double encoderAdjustment = ((Math.pow(gamepad2.right_stick_x, 2) * 0.93) + 0.07);
        double turnTableRotation;
        if (gamepad2.right_stick_x > 0.01 && attachmentActions.tableencodercount() < 3932) {
            turnTableRotation = encoderAdjustment;
        } else if (gamepad2.right_stick_x < -0.01 && attachmentActions.tableencodercount() > -5898) {
            turnTableRotation = -1.0 * encoderAdjustment;
        } else {
            turnTableRotation = 0;
        }
        return turnTableRotation;
    }
}
