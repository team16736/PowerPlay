
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
//import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

@TeleOp(name = "Main Tele Op", group = "Linear Opmode")
public class MainTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
//    private GyroActions gyroActions = null;
    private EncoderActions encoderActions = null;
    boolean memoryBit;
    boolean spinLeft;
    boolean placeBit;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
//        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);

        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        int currentTicks = 0;
        int speeding = 0;
        double speed = 0.8;
        double y = 0;
        double turnTableRotation = 0;
        double encoderAdjustment;
        double speedY; //Create new double for the speed.
        int currentPos; //Create an integer for the current position (IMPORTANT THAT ITS AN INTEGER, WILL NOT WORK OTHERWISE)
        boolean memBitLift = true;
        int gravityThresholdLift = -250;

        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveActions.setSpeed(0.4);


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

            attachmentActions.setLiftLevel(gamepad2.dpad_down, gamepad2.dpad_left || gamepad2.dpad_right, gamepad2.dpad_up);

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

            if((Math.abs(gamepad2.left_stick_y) < 0.01) && (attachmentActions.scissorLift1.getCurrentPosition() > gravityThresholdLift) && !memBitLift){
                attachmentActions.liftScissor(3000, -attachmentActions.scissorLift1.getCurrentPosition(), true);
                memBitLift = true;
            }

            //gamepad 2 right joystick is giving wonky values when negative. Need to switch gamepads or joysticks to adjust
            encoderAdjustment = ((Math.pow(gamepad2.right_stick_x, 2) * 0.93) + 0.07);
            if (gamepad2.right_stick_x > 0.01 && attachmentActions.tableencodercount() < 3932) {
                turnTableRotation = encoderAdjustment;
            } else if (gamepad2.right_stick_x < -0.01 && attachmentActions.tableencodercount() > -3932) {
                turnTableRotation = -1.0 * encoderAdjustment;
            } else {
                turnTableRotation = 0;
            }
            attachmentActions.turnTable.setPower(turnTableRotation);

            if (gamepad2.x) {
                attachmentActions.closeGripper();
            }
            if (gamepad2.y) {
                attachmentActions.openGripper();
            }

            changeSpeed(driveActions, gamepad1.dpad_up || gamepad1.x, gamepad1.dpad_down || gamepad1.b, gamepad1.a, gamepad1.y);


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
            }
            if (placeBit) {
                placeConeOnJunction(attachmentActions, gyroActions, encoderActions, spinLeft, HIGH);
            }*/

//
//            telemetry.addData("Current Position ", attachmentActions.slideTurnMotor.getCurrentPosition());
//            telemetry.addData("Target Position", currentTicks);
//            telemetry.addData("Current Power", attachmentActions.slideTurnMotor.getPower());
//            telemetry.update();
        }

        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
}
