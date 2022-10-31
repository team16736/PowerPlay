
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;

@TeleOp(name = "Experiment Tele Op", group = "Linear Opmode")

public class ExperimentTeleOp extends HelperActions {

    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    boolean memoryBit;
    boolean memBitArmSpin;

    @Override
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);

        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        double carouselPower = 0.4;
        int targetArmSpin = 0;
        int speeding = 0;
        double speed = 0.8;
        double speedY; //Create new double for the speed.
        int currentPos; //Create an integer for the current position (IMPORTANT THAT ITS AN INTEGER, WILL NOT WORK OTHERWISE)
        int armUpPosition1 = 0;
        int armUpPosition2 = 0;
        final float[] hsvValues = new float[3];

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
            if (gamepad1.x){
                driveActions.leftFront.setPower(0.5);
            }
            if (gamepad1.y){
                driveActions.rightFront.setPower(0.5);
            }
            if (gamepad1.b){
                driveActions.rightRear.setPower(0.5);
            }
            if (gamepad1.a){
                driveActions.leftRear.setPower(0.5);
            }

            double armSpeed = changeSpeedArm(gamepad2.dpad_up, gamepad2.dpad_down);

            speedY = gamepad2.left_stick_y; //map double speedY to the Y axis of player 1's left joystick.

            changeSpeed(driveActions, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a, gamepad1.x, gamepad1.y, gamepad1.b);

            telemetry.addData("Target Position", targetArmSpin);
            telemetry.addData("Memory Bit", memBitArmSpin);
            telemetry.addData("Position 1", armUpPosition1);
            telemetry.addData("Position 2", armUpPosition2);
            detectColor();
            telemetry.update();
        }
        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
    private void detectColor(){
        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.RGBToHSV(attachmentActions.boundaryDetector.red() * 8, attachmentActions.boundaryDetector.green() * 8, attachmentActions.boundaryDetector.blue() * 8, hsvValues);

        telemetry.addLine()
                .addData("Red", attachmentActions.boundaryDetector.red())
                .addData("Green", attachmentActions.boundaryDetector.green())
                .addData("Blue", attachmentActions.boundaryDetector.blue());
        telemetry.addLine()
                .addData("Hue", hsvValues[0])
                .addData("Saturation", hsvValues[1])
                .addData("Value", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if((attachmentActions.boundaryDetector.red()>60) || (attachmentActions.boundaryDetector.green()>60) || (attachmentActions.boundaryDetector.blue()>60)){
            telemetry.addData("Tape"," ");
        }else {
            telemetry.addData("No Tape", " ");
        }
    }
}
