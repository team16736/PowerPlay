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
    int currentPosition = 0;
    double currentVelocity = 0.0;

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
            minSpeed = 0.1 * topSpeed;


            telemetry.addData(">", "Press Play to start op mode");
            int totalTicks = (int) -(0.1161 * Math.pow(scissorDistance, 3) - 2.2579 * Math.pow(scissorDistance, 2) + 56.226 * scissorDistance + 36.647); //turn inches into ticks

            attachmentActions.liftScissor(3000, -totalTicks, true); //Lift the lift to specified height

            RobotLog.dd("FindJunction", "targetPos %f", targetPos);
            state = 1;
        }

        int localPos = encoderActions.motorFrontL.getCurrentPosition(); //Get the position only once per cycle
        double localVel = encoderActions.motorFrontL.getVelocity();
        if (steadyTable) {
            attachmentActions.turnTableEncoders(tableDegrees, false);
        }
        if (localPos != 0) {
            currentPosition = localPos;
            currentVelocity = localVel;
        } else {
            RobotLog.dd("FindJunction", "Got Zero");
        }
        int minCounter = 1;
        if (Math.abs(currentPosition) < Math.abs(targetPos) && opModeObj.opModeIsActive() && state == 1) {
            //If we aren't at the target position and the program is running
            RobotLog.dd("FindJunction", "Ramping");
            if (Math.abs(currentPosition) < Math.abs(targetPos - targetPos * adj)) {
                speed = topSpeed; //If less than 25% of the way, go to top speed
            } else {
                //Otherwise, ramp down to 5% speed over the rest of the distance
                drive = Math.abs(currentPosition) - (targetPos * (1 - adj));
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
            RobotLog.dd("FindJunction", "get tilt error");
            tiltError = gyroActions.getSteeringCorrection(degrees, Math.abs(speed) * 0.05, Math.abs(speed)); //Adjustment to go straight
            RobotLog.dd("FindJunction", "Set Motor Velocity");
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


            RobotLog.dd("FindJunction", "getting sensor distance");
            double distanceS1 = s1.getSensorDistance(DistanceUnit.MM); //Get the distance only once per cycle
            RobotLog.dd("FindJunction", "distance: %f currentPos: %d, counter: %d, time: %d, velocity: %f, heading error %f", distanceS1, currentPosition, counter, System.currentTimeMillis(), currentVelocity, tiltError / (speed * 0.05));
            if (distanceS1 > 2000) { //System for throwing out errors. If it goes from more than 2000 to less than 2000 to more again in less than three cycles, we know it's an error
                if (idCounter <= 2) {
                    dist = prevDist;
                    ticksAtLowestDist = prevDistTicks;
                }
                idCounter = 0;
            }
            if (distanceS1 > minDistance && idCounter > minCounter) {
                counter = minCounter + 1;
            }

            if (attachmentActions.scissorLift1.getCurrentPosition() < -490 && (Math.abs(targetPos) - Math.abs(currentPosition)) < 400 && distanceS1 < minDistance) {
                //If our scissor lift is high enough to see and we're far enough that we wouldn't see the other poles and we're seeing a pole
                if (idCounter == 0) { //Part of the error correction system.
                    prevDist = dist;
                    prevDistTicks = ticksAtLowestDist;
                }
                idCounter++;

                if (distanceS1 < dist) { //If the current distance is less than the past minimum, it's the new minimum and we record where we were when we got it

                    dist = distanceS1;
                    ticksAtLowestDist = currentPosition;

                }

                if (distanceS1 < minDistance) { //If we've seen the pole

                    memBitOn = true;

                }

                if ((memBitOn == true) && (distanceS1 > dist)) { //If we've seen the pole and our distance is above the minimum we've recorded, increment the counter

                    counter = counter + 1;

                } else {
                    counter = 0;
                }

                if (counter > minCounter) { //If the counter is more than 2, we know that we've been right in front of the pole, so we get out of this part and go into the others

                    targetPos = ticksAtLowestDist;

                    encoderActions.setVelocity(0, 0, 0, 0);

                }

                telemetry.addData("Scissor Y", attachmentActions.scissorLift1.getCurrentPosition());
                telemetry.addData("Scissor Y Target", totalTicks);

            }

        } else if (state == 1) { //If we've seen the goal or reached our set position
            if (counter <= minCounter && keptGoingCounter < 6) { //If we've reached the set position but haven't seen the pole, we try again but go a little further
                if (targetPos < 0) {
                    targetPos -= 60;
                } else {
                    targetPos += 60;
                }
                keptGoingCounter++;
                RobotLog.dd("FindJunction", "Kept Going");
                return;
            } else {
                state = 2;
            }
        }

        if (state == 2) { //If we've seen the goal
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
            if (placeState == 0) { //If it's done, reset it
                state = 0;
            } else {
                placeOnJunction(dist, ticksAtLowestDist, degrees, turnTableLeft, direction, offset); //Method for placing the cone on the junction according to where we are now and where we were when we saw it and how far away it was when we saw it
            }
        }
        telemetry.update();
    }

    public boolean placeOnJunction (double sensorDistance, int ticksAtLowestDist, double degrees, boolean turnTableLeft, int direction, double offset2) {
        if (placeState == 0) {
            if (sensorDistance > minDistance) { //If something's gone wrong and we haven't seen the junction, we give up and just drop the cone
                placeState = 4;
                return true;
            }
            int turnTableLefti = 1;
            int strafeSpeed = 700;
            if (!turnTableLeft) {turnTableLefti = -1;}
            overshoot = encoderActions.motorFrontL.getCurrentPosition() - ticksAtLowestDist;
//            if (overshoot < 0) {
//                overshoot += 1;
//            } else {
//                overshoot -= 1;
//            }
            RobotLog.dd("FindJunction:", "overshoot %d", overshoot);
            //Math for getting the distance to strafe and drive
            if (direction == HelperActions.FORWARDS) {
                strafe = (sensorDistance - sensorToJunction) * turnTableLefti / DistanceUnit.mmPerInch; //Take the distance we read minus the distance we want it to be away, convert to inches, and if the turntable is on the right, negate it
                double offset = 0.0 + offset2;
                if (ticksAtLowestDist > 0) {
                    offset *= -1.0;
                }
                drive = overshoot / 31.0 + offset; //Take the distance we went further than when we were looking at it, convert to inches, and add an offset
                //Basically the same process with all the other directions, just in different directions
            } else if (direction == HelperActions.BACKWARDS) {
                strafe = (sensorDistance - sensorToJunction) * -turnTableLefti / DistanceUnit.mmPerInch;
                double offset = -0.85 + offset2;
                drive = overshoot / 31.0 + offset;
            } else if (direction == HelperActions.RIGHT) {
                double offset = 0.5 + offset2;
                if (ticksAtLowestDist < 0) {
                    offset *= -1.0;
                }
                drive = (sensorDistance - sensorToJunction) * -turnTableLefti / DistanceUnit.mmPerInch + offset;
                strafe = overshoot / 31.0;
                driveSpeed = 350;
                strafeSpeed = 350;
            } else if (direction == HelperActions.LEFT) {
                double offset = 0.0 + offset2;
                if (ticksAtLowestDist < 0) {
                    offset *= -1.0;
                }
                drive = ((sensorDistance - sensorToJunction) * turnTableLefti) / DistanceUnit.mmPerInch + offset;
                strafe = (overshoot / 33.6)-1.0;
                driveSpeed = 350;
                strafeSpeed = 350;
            }
            boolean encoderMoveLeft = true;
            if (strafe < 0) {
                encoderMoveLeft = false;
                strafe = Math.abs(strafe);
            }
            RobotLog.dd("FindJunction", "Strafe %f, drive %f, sensordistance %f, distance at minimum %d, current distance %d", strafe, drive, sensorDistance, ticksAtLowestDist, encoderActions.motorFrontL.getCurrentPosition());
            encoderActions.encoderStrafeNoWhile(strafeSpeed, strafe, encoderMoveLeft); //Strafe the distance set above
            placeState = 1;
        }
        if (placeState == 1 && !encoderActions.motorFrontL.isBusy()) {
            placeState = 2;
        }
        if (placeState == 2) {
            gyroActions.encoderGyroDriveStateMachine(driveSpeed, -drive, degrees); //Drive the distance set above
            placeState = 3;
        }
        if (placeState == 3) {
            if (!gyroActions.encoderGyroDriveStateMachine(driveSpeed, -drive, degrees)){ //Continue driving the distance set above
                placeState = 4;
            }
        }
        if (placeState == 4) {
            //Set the cone down
            attachmentActions.liftScissor(3000, -1.5, false);
            attachmentActions.openGripper();
//            attachmentActions.liftScissor(3000, -1.5, false);
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
        topSpeed = 2400 * 0.25; // = 1200
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