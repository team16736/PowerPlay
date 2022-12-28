package org.firstinspires.ftc.teamcode.actions;//package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FindJunctionAction {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DriveActions driveActions;
    private AttachmentActions attachmentActions;
    private DistanceSensorActions s1;
    private EncoderActions encoderActions;
    private GyroActions gyroActions;
    private static LinearOpMode opModeObj;

    double sensorToCone = 96.8375; // distance from sensor to cone\

    public int state;
    public int placeState;
    double speed;
    double targetPos;
    double degrees;
    double tableDegrees;
    boolean memBitOn;
    double topSpeed;
    double ramp;
    double tiltError;
    double minSpeed;
    double x;
    double dur;
    double adj;
    double dist;
    int ticksAtLowestDist;
    int counter;
    int totalTicks;
    int overshoot;

    double strafe;
    double drive;

    ElapsedTime runtime = new ElapsedTime();

    public FindJunctionAction(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, DriveActions driveActions, AttachmentActions attachmentActions, DistanceSensorActions distanceSensorActions, EncoderActions encoderActions, GyroActions gyroActions) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.driveActions = driveActions;
        this.attachmentActions = attachmentActions;
        this.s1 = distanceSensorActions;
        this.encoderActions = encoderActions;
        this.gyroActions = gyroActions;
        opModeObj = opMode;
    }

    public void findJunction(double distance, double scissorDistance) {
        findJunctionStateMachine(distance, scissorDistance, true);
        while (state != 0) {
            findJunctionStateMachine(distance, scissorDistance, true);
        }
    }
    public void findJunctionStateMachine(double distance, double scissorDistance, boolean steadyTable) {
        //Telemetry telemetry;
        telemetry.addData("state", state);
        telemetry.addData("placeState", placeState);
        if (state == 0) {
            reset();

            driveActions.setMotorDirection_Forward();
            encoderActions.resetEncoder();

            encoderActions.motorFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderActions.motorFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderActions.motorBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderActions.motorBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runtime.reset();

            targetPos = distance * 32.3;
            degrees = gyroActions.getRawHeading() - gyroActions.headingOffset;
            tableDegrees = attachmentActions.getTurntablePosition();
            if (distance < 0) {
                topSpeed *= -1;
            }
            minSpeed = 0.05 * topSpeed;


            telemetry.addData(">", "Press Play to start op mode");
            int totalTicks = (int) -(0.1161 * Math.pow(scissorDistance, 3) - 2.2579 * Math.pow(scissorDistance, 2) + 56.226 * scissorDistance + 36.647);

            attachmentActions.liftScissor(3000, -totalTicks, true);
            state = 1;
        }

        if (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < Math.abs(targetPos) && opModeObj.opModeIsActive() && state == 1) {
            if (steadyTable) {
                attachmentActions.turnTableEncoders(tableDegrees, false);
            }
            if (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < Math.abs(targetPos - targetPos * adj)) {
                speed = topSpeed;
            } else {
                drive = encoderActions.motorFrontL.getCurrentPosition() - (targetPos * (1 - adj));
                //ramp = topSpeed * 2 * ((targetPos-motorFrontL.getCurrentPosition())/targetPos);
//                ramp = topSpeed * (targetPos - encoderActions.motorFrontL.getCurrentPosition()) / (targetPos - (targetPos * adj));
                ramp = drive * -(topSpeed - minSpeed) / (adj * targetPos) + topSpeed;
                if (Math.abs(ramp) < Math.abs(minSpeed)) {
                    ramp = minSpeed;
                }
                speed = ramp;
            }
            tiltError = gyroActions.getSteeringCorrection(degrees, Math.abs(speed) * 0.05, Math.abs(speed));
            encoderActions.setVelocity(speed - tiltError, speed + tiltError, speed - tiltError, speed + tiltError);
//            encoderActions.setVelocity(speed, speed, speed, speed);


            double distanceS1 = s1.getSensorDistance(DistanceUnit.MM);
            int currentPos = encoderActions.motorFrontL.getCurrentPosition();
            if (attachmentActions.scissorLift1.getCurrentPosition() < -490 && Math.abs(targetPos - currentPos) < 600 && distanceS1 < 200) {


                if (distanceS1 < dist) {

                    dist = distanceS1;
                    ticksAtLowestDist = currentPos;

                }

                if (distanceS1 < 200) {

                    memBitOn = true;

                }

                if ((memBitOn == true) && (distanceS1 > dist)) {

                    counter = counter + 1;

                } else {
                    counter = 0;
                }

                if (counter > 2) {

                    targetPos = ticksAtLowestDist;

                    encoderActions.setVelocity(0, 0, 0, 0);

                }

                telemetry.addData("Scissor Y", attachmentActions.scissorLift1.getCurrentPosition());
                telemetry.addData("Scissor Y Target", totalTicks);

            }

        } else if (state == 1) {
            state = 2;
        }

        if (state == 2) {
            encoderActions.setVelocity(0, 0, 0, 0);

            dur = runtime.time();

            overshoot = encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist;
            telemetry.addData("minimum distance", dist);
            telemetry.addData("Motor ticks at lowest dist", ticksAtLowestDist);
            telemetry.addData("Current pos", encoderActions.motorFrontL.getCurrentPosition());
            telemetry.addData("Distance past lowest dist", overshoot);
            telemetry.addData("time", dur);
            telemetry.addData("counter", counter);

            state = 3;
        }
        if (state == 3 && !placeOnJunction(dist, ticksAtLowestDist, degrees)) {
            state = 0;
        }
        telemetry.update();
    }

    public boolean placeOnJunction (double sensorDistance, int ticksAtLowestDist, double degrees) {
        if (placeState == 0) {
            strafe = sensorDistance - sensorToCone;
            overshoot = encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist;
            double offset = 0.85;
            if (ticksAtLowestDist < 0) {
                offset *= -1;
            }
            drive = overshoot / 31 + offset;
            encoderActions.encoderStrafeNoWhile(700, strafe / DistanceUnit.mmPerInch, false);
            placeState = 1;
        }
        if (placeState == 1 && !encoderActions.motorFrontL.isBusy()) {
            placeState = 2;
        }
        if (placeState == 2) {
            if (!gyroActions.encoderGyroDriveStateMachine(350, -drive, degrees)){
                placeState = 3;
            }
        }
        if (placeState == 3) {
            attachmentActions.openGripper();
            attachmentActions.liftScissor(3000, -1.5, false);
            placeState = 0;
            telemetry.addData("strafe", strafe);
            telemetry.addData("drive", drive);
            telemetry.addData("overshoot", overshoot);
            telemetry.addData("ticks at lowest distance", ticksAtLowestDist);
            telemetry.addData("current ticks", encoderActions.motorFrontL.getCurrentPosition());
            return false;
        }
        return true;
    }
    public void reset() {
        placeState = 0;
        topSpeed = 2400 * 0.5; // = 1200
        state = 0;
        speed = 0;
        memBitOn = false;
        dur = 0.0;
        adj = 0.75;
        dist = 2000.0;
        ticksAtLowestDist = 0;
        counter = 0;
        ramp = 0.0;
    }
}