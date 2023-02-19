package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindImageOnCone;
import org.firstinspires.ftc.teamcode.actions.FindJunctionAction;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.constants.MotorConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.HelperActions;

//moves forward to the carousel, spins it, then turns and parks in the storage unit

@Autonomous(name = "Test :)")
public class AutonomousTest extends HelperActions{
//    private DriveActions driveActions = null;
    private AttachmentActions attachmentActions = null;
    private EncoderActions encoderActions = null;
    private GyroActions gyroActions = null;
//    private FindImageOnCone findImageOnCone = null;
    private DistanceSensorActions s1 = null;

    int state = 0;
    double startTime;
    public int coneNum = 5;
    double distanceFromCones;
    double ticksAtDistance;
    boolean distanceMemBit = false;
    int strafeSpeed;
    double distance = 0;

    public void runOpMode() {

//        driveActions = new DriveActions(telemetry, hardwareMap);
        attachmentActions = new AttachmentActions(telemetry, hardwareMap);
        encoderActions = new EncoderActions(this, telemetry, hardwareMap);
        gyroActions = new GyroActions(this, telemetry, hardwareMap);
//        findImageOnCone = new FindImageOnCone(telemetry, hardwareMap);
        s1 = new DistanceSensorActions(hardwareMap, 0.5, 10, ConfigConstants.BASE_RANGE);
//        driveActions.setMotorDirection_Forward();
//        FindJunctionAction findJunctionAction = new FindJunctionAction(hardwareMap, telemetry, this, driveActions, attachmentActions, s1, encoderActions, gyroActions);

        boolean memBitOn = false;
        boolean memBitOff = false;
        int ticks = 0;
        int ticksOff = 0;


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
//        if (opModeIsActive()) {

            attachmentActions.turnTableEncoders(10, 0.0019, 0, 0.17, 0.5);

//            attachmentActions.turnTableEncoders(0, false);
//            gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//            while (!attachmentActions.isDone || findJunctionAction.state != 0) {
//                attachmentActions.turnTableEncoders(0, false);
//                if (gyroActions.driveState != 0) {
//                    gyroActions.encoderGyroDriveStateMachine(700, 32, 0);
//                }
//            }

//            double total = 0;
//            double avg;
//            for (int i = 0; i < 200; i++) {
//                avg = s1.getSensorDistance();
//                total += avg;
//                RobotLog.dd("FindJunction", "%f", avg);
//            }

//            James debug code
//            attachmentActions.closeGripper();
//            sleep(500);
//            findJunctionAction.findJunction(48,24, false, RIGHT);
//
//            The static code for goToCone
//            encoderActions.encoderSpinNoWhile(300, -90, true);
//            sleep(500);
//            attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            attachmentActions.liftScissor(3000, 290, true);
//            attachmentActions.turnTableEncoders(-90, false);
//            while (!attachmentActions.isDone) {
//                attachmentActions.turnTableEncoders(-90, false);
//            }
//            gyroActions.encoderGyroDrive(700, 47.5, -90);
//            attachmentActions.closeGripper();
//            sleep(500);
//            attachmentActions.liftScissor(3000, 24, false);
//            attachmentActions.turnTableEncoders(0, false);


//            sleep(3000);
//            encoderActions.encoderSpin(300, 90, false);
//            for (int i = 0; i < 5; i++) {
////                encoderActions.encoderStrafe(700, 40, false);
//                gyroActions.encoderGyroDrive(700, 60, 0);
//                sleep(500);
////                encoderActions.encoderStrafe(700, 40, true);
//                gyroActions.encoderGyroDrive(700, -40, -90);
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
    double headingError;
    private void setSpeed(double speed, double heading) {
        headingError = gyroActions.getSteeringCorrection(heading, speed * 0.05, speed);
        if (distance < 0) {
            headingError *= -1;
        }
        encoderActions.motorFrontL.setVelocity(speed - headingError);
        encoderActions.motorFrontR.setVelocity(speed + headingError);
        encoderActions.motorBackL.setVelocity(speed - headingError);
        encoderActions.motorBackR.setVelocity(speed + headingError);
    }
    private void initDrive() {
        int totalTicks = (int) (31 * distance);
        encoderActions.motorFrontL.setTargetPosition(totalTicks);
        encoderActions.motorFrontR.setTargetPosition(totalTicks);
        encoderActions.motorBackL.setTargetPosition(totalTicks);
        encoderActions.motorBackR.setTargetPosition(totalTicks);
    }
//    private void goToCone() {
//        while (state != 6) {
//            if (state == 0) {
//                encoderActions.encoderSpinNoWhile(300, -90, true);
//                attachmentActions.scissorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                attachmentActions.scissorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                attachmentActions.liftScissor(3000, 290, true);
//                state = 1;
//            }
//            if (state == 1 && !encoderActions.motorFrontL.isBusy()) {
//                state = 2;
//            }
//            if (state == 2) {
//                gyroActions.encoderGyroDrive(700, 47, -90);
//                if (gyroActions.driveState == 0) {
//                    state = 3;
//                }
//            }
//            if (state == 3 && attachmentActions.isDone) {
//                attachmentActions.closeGripper();
//                startTime = System.currentTimeMillis();
//                state = 4;
//            }
//            if (state == 4 && System.currentTimeMillis() - startTime > 650) {
//                attachmentActions.liftScissor(3000, 24, false);
//                state = 5;
//            }
//            if (state == 5 && attachmentActions.getLiftHeight() > 10) {
//                state = 6;
//            }
//            attachmentActions.turnTableEncoders(-90, false);
//        }
//    }
}
