package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        driveActions.setMotorDirection_Forward();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            double speed = 762.2;
            //encoderActions.encoderDriveSpeedRamp(speed, 60, 3);
//            gyroActions.runUsingEncoders();

            attachmentActions.turnTableEncoders(20, 0.2, this);
            sleep(10000);
//            gyroActions.gyroSpin(0.2, 90.0);
        }
    }
    private void placeBlock(EncoderActions encoderActions, AttachmentActions attachmentActions, int blockPlace) {
    }
}
