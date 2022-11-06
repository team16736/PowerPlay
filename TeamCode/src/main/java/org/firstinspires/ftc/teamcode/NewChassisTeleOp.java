
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;
import org.firstinspires.ftc.teamcode.actions.NewDriveActions;

@TeleOp(name = "New Chassis TeleOp", group = "Linear Opmode")
public class NewChassisTeleOp extends HelperActions {

    private NewDriveActions newDriveActions = null;
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;

    @Override
    public void runOpMode() {

        newDriveActions = new NewDriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        driveActions = new DriveActions(telemetry, hardwareMap);

        //Set Speed for teleOp. Mecannum wheel speed.
        //newDriveActions.setSpeed(1.0);

        double carouselPower = 0.4;
        int currentTicks = 0;
        int speeding = 0;
        double speed = 0.8;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        newDriveActions.setSpeed(0.8);

        while (opModeIsActive()) {
            //TODO: add functionality for red side carousel

            /** Gamepad 1 **/
            newDriveActions.drive(
                    gamepad1.left_stick_x,      //joystick controlling strafe
                    -gamepad1.left_stick_y,     //joystick controlling forward/backward
                    gamepad1.right_stick_x);    //joystick controlling rotation
            if (gamepad2.x){
                attachmentActions.closeGripper();
            }
            if (gamepad2.y){
                attachmentActions.openGripper();
            }

            changeSpeed(driveActions, gamepad1.dpad_up || gamepad1.x, gamepad1.dpad_down || gamepad1.b, gamepad1.a, gamepad1.y);

            telemetry.addData("Target Position", currentTicks);
            telemetry.update();
        }

        telemetry.addData("[ROBOTNAME] ", "Going");
        telemetry.update();

        idle();
    }
}
