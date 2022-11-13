
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;

@TeleOp(name = "Main Tele Op", group = "Linear Opmode")
public class MainTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    boolean memoryBit;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);

        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        int currentTicks = 0;
        int speeding = 0;
        double speed = 0.8;
        double y = 0;
        double x = 0;
        double speedY; //Create new double for the speed.
        int currentPos; //Create an integer for the current position (IMPORTANT THAT ITS AN INTEGER, WILL NOT WORK OTHERWISE)


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveActions.setSpeed(0.8);


        while (opModeIsActive()) {

            /** Gamepad 1 **/
            driveActions.drive(
                    (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * 0.5),      //joystick controlling strafe
                    (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * 0.5),     //joystick controlling forward/backward
                    (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * 0.5));    //joystick controlling rotation
//            telemetry.addData("Left stick x", gamepad1.left_stick_x);
//            telemetry.addData("left stick y", gamepad1.left_stick_y);
//            telemetry.addData("right stick x", gamepad1.right_stick_x);
//            telemetry.update();
            y = gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
            attachmentActions.scissorLift1.setPower(y);
            attachmentActions.scissorLift2.setPower(y);

            if (gamepad2.right_stick_x > 0.01 && attachmentActions.tableencodercount() < 3932) {
                x = ((Math.abs(Math.pow(gamepad2.right_stick_x, 2)) * 0.93) + 0.07);
            } else if (gamepad2.right_stick_x < -0.01 && attachmentActions.tableencodercount() > -3932) {
                x = -((Math.abs(Math.pow(gamepad2.right_stick_x, 2)) * 0.93) + 0.07);
            } else {
                x = 0;
            }
            attachmentActions.turnTable.setPower(x);

            if (gamepad2.x) {
                attachmentActions.closeGripper();
            }
            if (gamepad2.y) {
                attachmentActions.openGripper();
            }

            double armSpeed = changeSpeedArm(gamepad2.dpad_up, gamepad2.dpad_down);

            speedY = gamepad2.left_stick_y; //map double speedY to the Y axis of player 1's left joystick.
//            currentPos = slideExtendMotor.getCurrentPosition(); //map integer currentPos to the arm's current extended position.
//            if(currentPos <= -3450 && speedY < 0) {//limit extending to -3450 encoder ticks, about 1 inch from fully extended. check if you are pressing up to continue extending, if so then set speed to 0.
//                speedY = 0;
//            }
//            if(currentPos >= 0 && speedY > 0) {//limit retracting to 0 ticks, fully closed. check if pressing down on joystick, if so set speed to 0.
//                speedY = 0;
//                currentPos = 0; //Set the arm to go to the fully closed position. Not needed (dont quote me on this)
//            }
//            if((speedY == 0) && (!memoryBit)) { //Only runs if speedY is 0 (joystick idle) and the memory bit is false.
//                slideExtendMotor.setTargetPosition(currentPos); //Set the arm to hold its position.
//                slideExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Set the arm's run mode so it actually does stuff
//                slideExtendMotor.setPower(0.5); //Set the motor to half power.
//                memoryBit = true; //Change the memory bit back to true so it only runs once.
//            }
//            if((speedY != 0)) { //If the joystick IS being pushed, run this code.
//                memoryBit = false; //Set memory bit to false so that the previous if statement works.
//                slideExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS); //Change mode to run using encoders.
//                slideExtendMotor.setPower(speedY*(armSpeed/.6)); //Set the motor power to the current joystick position.
//            }
//
//            telemetry.addData("Gamepad is at", speedY); //testing junk
//            telemetry.addData("Arm is extended to", slideExtendMotor.getCurrentPosition()); //testing junk
//            if(Math.abs(gamepad2.right_stick_y)>0.1){
//                attachmentActions.slideTurnMotor.setPower(gamepad2.right_stick_y * -armSpeed);
//                currentTicks = attachmentActions.slideTurnMotor.getCurrentPosition();
//            }else if(attachmentActions.slideTurnMotor.getCurrentPosition() < currentTicks){
//                attachmentActions.slideTurnMotor.setPower((attachmentActions.slideTurnMotor.getCurrentPosition()-currentTicks)*-0.003);
//            }else if(attachmentActions.slideTurnMotor.getCurrentPosition() > currentTicks){
//                attachmentActions.slideTurnMotor.setPower((attachmentActions.slideTurnMotor.getCurrentPosition()-currentTicks)*-.001);
//            }
//
            changeSpeed(driveActions, gamepad1.dpad_up || gamepad1.x, gamepad1.dpad_down || gamepad1.b, gamepad1.a, gamepad1.y);
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
