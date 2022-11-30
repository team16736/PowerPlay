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

@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends HelperActions{
    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
    private FindImageOnCone findImageOnCone = null;
    public void runOpMode() {

        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        driveActions.setMotorDirection_Forward();
        attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        boolean memBitOn = false;
        boolean memBitOff = false;
        int ticks = 0;
        int ticksOff = 0;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
//        if (opModeIsActive()) {
            double speed = 762.2;
            double degrees = -20;
            findImageOnCone.findObject();
            //encoderActions.encoderDriveSpeedRamp(speed, 60, 3);
//            gyroActions.runUsingEncoders();
//            attachmentActions.liftScissor(2000, 10, false);
//            attachmentActions.closeGripper();
//            sleep(500);
//            attachmentActions.liftScissor(3000, 5, false);
//            while (attachmentActions.scissorLift1.isBusy()) {}
//            attachmentActions.turnTableEncoders(degrees, 0.00044, 0.00000016, 0.5);
//            while (!attachmentActions.isDone) {
//                attachmentActions.turnTableEncoders(degrees, 0.00044, 0.00000016, 0.5);
//            }
//            sleep(10000);
//            attachmentActions.turnTableEncoders(7.5, 0.2);
//            for (int i = 0; i < 5; i++) {
//                encoderActions.encoderDrive(330,36);
//                sleep(500);
//                encoderActions.encoderDrive(330,-36);
//                sleep(500);
//            }
            /*if(!memBitOff) {
                attachmentActions.turnTable.setPower(0.1);
                memBitOff = true;
            }
            if(attachmentActions.getJunctionDistance() < 200 && !memBitOn){
                ticks = attachmentActions.tableencodercount();
                memBitOn = true;
                telemetry.addData("ticks on", ticks);
                telemetry.update();
            }
            if(attachmentActions.getJunctionDistance() > 200 && memBitOn){
                ticksOff = attachmentActions.tableencodercount();
                memBitOn = false;
                attachmentActions.turnTable.setPower(0);
                telemetry.addData("ticks off", ticksOff);
                telemetry.update();
            }*/
//            attachmentActions.extendGripper(5);
//            while (attachmentActions.getJunctionDistance() < 1000) {}

//            telemetry.addData("distance", attachmentActions.getJunctionDistance());
//            telemetry.addData("ticks on", ticks);
//            telemetry.addData("ticks off", ticksOff);
//            telemetry.addData("difference", ticks - ticksOff);
//            telemetry.update();
//            gyroActions.gyroSpin(0.2, 90.0);
        }
    }
}
