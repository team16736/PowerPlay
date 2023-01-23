package org.firstinspires.ftc.teamcode.actions;//package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

public class FindJunctionAction {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DriveActions driveActions;
    private AttachmentActions attachmentActions;
    private DistanceSensorActions s1;
    private EncoderActions encoderActions;
    private GyroActions gyroActions;
    private static LinearOpMode opModeObj;

    double sensorToJunction = 146.05; // distance from sensor to cone\

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
    int driveSpeed = 350;
    int idCounter;
    double prevDist;
    int prevDistTicks;
    int keptGoingCounter;
    int minDistance = 300;

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

    //turnTableLeft is whether the turntable is left from the perspective of the direction the robot is driving
    public void findJunction(double distance, double scissorDistance, boolean turnTableLeft, int direction) {
        findJunctionStateMachine(distance, scissorDistance, true,  turnTableLeft, direction);
        while (state != 0) {
            findJunctionStateMachine(distance, scissorDistance, true, turnTableLeft, direction);
        }
    }
    public void findJunctionStateMachine(double distance, double scissorDistance, boolean steadyTable, boolean turnTableLeft, int direction) {
        findJunctionStateMachine(distance, scissorDistance, steadyTable, turnTableLeft, direction, 0, 0);
    }
    public void findJunctionStateMachine(double distance, double scissorDistance, boolean steadyTable, boolean turnTableLeft, int direction, double offset, double heading) {
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

            double ticksPerInch = 32.3;
            if (direction == HelperActions.RIGHT || direction == HelperActions.LEFT) {ticksPerInch = 33.6;}

            targetPos = distance * ticksPerInch;
            degrees = gyroActions.getRawHeading() - gyroActions.headingOffset;
            if (heading != 0) {
                degrees = heading;
            }
            tableDegrees = attachmentActions.getTurntablePosition();
            if (distance < 0) {
                topSpeed *= -1;
            }
            minSpeed = 0.05 * topSpeed;


            telemetry.addData(">", "Press Play to start op mode");
            int totalTicks = (int) -(0.1161 * Math.pow(scissorDistance, 3) - 2.2579 * Math.pow(scissorDistance, 2) + 56.226 * scissorDistance + 36.647);

            attachmentActions.liftScissor(3000, -totalTicks, true);
            RobotLog.dd("FindJunction", "targetPos %f", targetPos);
            state = 1;
        }

        int minCounter = 1;
        if (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < Math.abs(targetPos) && opModeObj.opModeIsActive() && state == 1) {
            if (steadyTable) {
                attachmentActions.turnTableEncoders(tableDegrees, false);
            }
            if (Math.abs(encoderActions.motorFrontL.getCurrentPosition()) < Math.abs(targetPos - targetPos * adj)) {
                speed = topSpeed;
            } else {
                drive = Math.abs(encoderActions.motorFrontL.getCurrentPosition()) - (targetPos * (1 - adj));
                //ramp = topSpeed * 2 * ((targetPos-motorFrontL.getCurrentPosition())/targetPos);
//                ramp = topSpeed * (targetPos - encoderActions.motorFrontL.getCurrentPosition()) / (targetPos - (targetPos * adj));
                ramp = drive * -(topSpeed - minSpeed) / (adj * targetPos) + topSpeed;
                if (Math.abs(ramp) < Math.abs(minSpeed)) {
                    ramp = minSpeed;
                }
                speed = ramp;
                if (targetPos < 0) {
                    ramp *= -1;
                }
            }
            tiltError = gyroActions.getSteeringCorrection(degrees, Math.abs(speed) * 0.05, Math.abs(speed));
            if (direction == HelperActions.FORWARDS) {
                encoderActions.setVelocity(speed - tiltError, speed + tiltError, speed - tiltError, speed + tiltError);
            } else if (direction == HelperActions.BACKWARDS) {
                encoderActions.setVelocity(-speed - tiltError, -speed + tiltError, -speed - tiltError, -speed + tiltError);
            } else if (direction == HelperActions.RIGHT) {
                encoderActions.setVelocity(speed - tiltError, -speed + tiltError, -speed - tiltError, speed + tiltError);
            } else if (direction == HelperActions.LEFT) {
                encoderActions.setVelocity(-speed - tiltError, speed + tiltError, speed - tiltError, -speed + tiltError);
            }
//            encoderActions.setVelocity(speed, speed, speed, speed);


            double distanceS1 = s1.getSensorDistance(DistanceUnit.MM);
            int currentPos = encoderActions.motorFrontL.getCurrentPosition();
            RobotLog.dd("FindJunction", "distance: %f currentPos: %d, counter: %d, time: %d, velocity: %f", distanceS1, currentPos, counter, System.currentTimeMillis(), encoderActions.motorFrontL.getVelocity());
            if (distanceS1 > 2000) {
                if (idCounter <= 2) {
                    dist = prevDist;
                    ticksAtLowestDist = prevDistTicks;
                } else {
                    counter = minCounter + 1;
                }
                idCounter = 0;
            }

            if (attachmentActions.scissorLift1.getCurrentPosition() < -490 && (Math.abs(targetPos) - Math.abs(currentPos)) < 400 && distanceS1 < minDistance) {
                if (idCounter == 0) {
                    prevDist = dist;
                    prevDistTicks = ticksAtLowestDist;
                }
                idCounter++;

                if (distanceS1 < dist) {

                    dist = distanceS1;
                    ticksAtLowestDist = currentPos;

                }

                if (distanceS1 < minDistance) {

                    memBitOn = true;

                }

                if ((memBitOn == true) && (distanceS1 > dist)) {

                    counter = counter + 1;

                } else {
                    counter = 0;
                }

                if (counter > minCounter) {

                    targetPos = ticksAtLowestDist;

                    encoderActions.setVelocity(0, 0, 0, 0);

                }

                telemetry.addData("Scissor Y", attachmentActions.scissorLift1.getCurrentPosition());
                telemetry.addData("Scissor Y Target", totalTicks);

            }

        } else if (state == 1) {
            if (counter <= minCounter && keptGoingCounter < 12) {
                targetPos += 30;
                keptGoingCounter++;
                RobotLog.dd("FindJunction", "Kept Going");
                return;
            } else {
                state = 2;
            }
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
            RobotLog.dd("FindJunction", "state 2, placestate %d", placeState);
            placeOnJunction(dist, ticksAtLowestDist, degrees, turnTableLeft, direction, offset);
//            telemetry.update();
//            opModeObj.sleep(1000000);

            state = 3;
        }
        if (state == 3) {
            if (placeState == 0) {
                state = 0;
            } else {
                placeOnJunction(dist, ticksAtLowestDist, degrees, turnTableLeft, direction, offset);
            }
        }
        telemetry.update();
    }

    public boolean placeOnJunction (double sensorDistance, int ticksAtLowestDist, double degrees, boolean turnTableLeft, int direction, double offset2) {
        if (placeState == 0) {
            if (sensorDistance > minDistance) {
                placeState = 4;
                return true;
            }
            int turnTableLefti = 1;
            int strafeSpeed = 700;
            if (!turnTableLeft) {turnTableLefti = -1;}
            overshoot = encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist;
            RobotLog.dd("FindJunction:", "overshoot %d", overshoot);
            if (direction == HelperActions.FORWARDS) {
                strafe = (sensorDistance - sensorToJunction) * turnTableLefti / DistanceUnit.mmPerInch;
                double offset = 1.0 + offset2;
                if (ticksAtLowestDist > 0) {
                    offset *= -1;
                }
                drive = overshoot / 31.0 + offset;
            } else if (direction == HelperActions.BACKWARDS) {
                strafe = (sensorDistance - sensorToJunction) * -turnTableLefti / DistanceUnit.mmPerInch;
                double offset = -0.85 + offset2;
                drive = overshoot / 31.0 + offset;
            } else if (direction == HelperActions.RIGHT) {
                double offset = 0.5 + offset2;
                if (ticksAtLowestDist < 0) {
                    offset *= -1;
                }
                drive = (sensorDistance - sensorToJunction) * -turnTableLefti / DistanceUnit.mmPerInch + offset;
                strafe = overshoot / 31.0 + 1;
                driveSpeed = 700;
                strafeSpeed = 350;
            } else if (direction == HelperActions.LEFT) {
                double offset = 0.0 + offset2;
                if (ticksAtLowestDist < 0) {
                    offset *= -1;
                }
                drive = ((sensorDistance - sensorToJunction) * turnTableLefti) / DistanceUnit.mmPerInch + offset;
                strafe = (overshoot / 33.6);
                driveSpeed = 700;
                strafeSpeed = 350;
            }
            boolean encoderMoveLeft = true;
            if (strafe < 0) {
                encoderMoveLeft = false;
                strafe = Math.abs(strafe);
            }
            RobotLog.dd("FindJunction", "Strafe %f, drive %f, sensordistance %f, distance at minimum %d, current distance %d", strafe, drive, sensorDistance, ticksAtLowestDist, encoderActions.motorFrontL.getCurrentPosition());
            encoderActions.encoderStrafeNoWhile(strafeSpeed, strafe, encoderMoveLeft);
            placeState = 1;
        }
        if (placeState == 1 && !encoderActions.motorFrontL.isBusy()) {
            placeState = 2;
        }
        if (placeState == 2) {
            gyroActions.encoderGyroDriveStateMachine(driveSpeed, -drive, degrees);
            placeState = 3;
        }
        if (placeState == 3) {
            if (!gyroActions.encoderGyroDriveStateMachine(driveSpeed, -drive, degrees)){
                placeState = 4;
            }
        }
        if (placeState == 4) {
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
        idCounter = 0;
        prevDist = 8190;
        prevDistTicks = 8190;
        keptGoingCounter = 0;
    }
}