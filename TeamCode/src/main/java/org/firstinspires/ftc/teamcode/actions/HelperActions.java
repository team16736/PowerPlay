package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;

public abstract class HelperActions extends LinearOpMode {

    protected ColorSensor right_sensor;
    protected ColorSensor left_sensor;
    protected boolean foundStone = false;
    protected float hsvValues[] = {0F, 0F, 0F};

    public final double SPEED = 0.5;

    public static int LEFT = 1;
    public static int RIGHT = 2;
    public static int FORWARDS = 3;
    public static int BACKWARDS = 4;
    public static int LOW = 5;
    public static int MEDIUM = 6;
    public static int HIGH = 7;

    private int speeding = 0;
    private double speed = 0.6;
    private int speedingArm = 0;
    private double speedArm = 0.6;

    //stuff for placing cone
    private int conePlacementState = 0;
    private boolean inCone;
    private double extenderRange = 0; //Range the extender can extend to. If it gets added back on, it's 6.25
    private double startingPosition;
    private boolean firstDetectedBit;
    private double startTime;
    private double totalDistance;
    private int i;
    private double distance;
    private boolean usingExtender;
    private boolean extendingBit;
    private double extendingTime;
    private double extendingSpeed = 200;
    private boolean releasingBit;
    private double releasingTime;
    public boolean isPlacingCone;

    //Chill method variables
    public int chillThreshold = 250;
    boolean upBit = false;
    boolean downBit = true;


    public double driveAndDetect(EncoderActions encoderActions, AttachmentActions attachmentActions, double speed, double distance) {
        encoderActions.resetEncoder();
        // Set the motor's target position to 6.4 rotations
        double ticksPerInch = 32.2;
        int totalTicks = (int) (ticksPerInch * distance);
        encoderActions.motorFrontL.setTargetPosition(totalTicks);
        encoderActions.motorFrontR.setTargetPosition(totalTicks);
        encoderActions.motorBackL.setTargetPosition(totalTicks);
        encoderActions.motorBackR.setTargetPosition(totalTicks);


        // Switch to RUN_TO_POSITION mode
        encoderActions.motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderActions.motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 1 revolution per second
        encoderActions.setVelocity(speed, speed, speed, speed);

        // While the Op Mode is running, show the motor's status via telemetry
        while (encoderActions.motorFrontL.isBusy() && encoderActions.motorFrontR.isBusy() && encoderActions.motorBackL.isBusy() && encoderActions.motorBackR.isBusy()) {
            if (attachmentActions.getJunctionDistance() < 10) {
                return attachmentActions.getJunctionDistance();
            }
        }
        return 0;
    }

    //    public void fancySpinRight(EncoderActions encoderActions, double speed, double distance){
//        encoderActions.fancySpin(speed, distance, false);
//    }
//    public void fancySpinLeft(EncoderActions encoderActions, double speed, double distance){
//        encoderActions.fancySpin(speed, distance, true);
//    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public void changeSpeed(DriveActions driveActions, boolean upOne, boolean downOne, boolean upTwo, boolean downTwo) {
        if (upOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.1;
            }
        }
        if (downOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.1;
            }
        }
        if (upTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.2;
            }
        }
        if (downTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.2;
            }
        }
        if (!upOne && !downOne && !upTwo && !downTwo) {
            speeding = 0;
        }
        if (speed < 0) {
            speed = 0;
        }
        if (speed > 1.0) {
            speed = 1.0;
        }
        driveActions.setSpeed(speed);
        telemetry.addData("speed: ", speed);
    }

    public double changeSpeedArm(boolean up, boolean down) {
        if (up) {
            speedingArm++;
            if (speedingArm == 1) {
                speedArm = speedArm + 0.1;
            }
        }
        if (down) {
            speedingArm++;
            if (speedingArm == 1) {
                speedArm = speedArm - 0.1;
            }
        }
        if (!up && !down) {
            speedingArm = 0;
        }
        if (speedArm < 0) {
            speedArm = 0;
        }
        if (speedArm > 1.0) {
            speedArm = 1.0;
        }
        return speedArm;
    }

    //Places the cone on the junction when the junction is within 100 degrees of the sensor.
//    public void placeConeOnJunction(AttachmentActions attachmentActions, GyroActions gyroActions, EncoderActions encoderActions, boolean spinLeft, int level) {
//        //Initializes the variables the first time the method is called
//        if (conePlacementState == 0) {
//            initConePlacement();
//            conePlacementState = 1;
//        }
//        //Center the turntable before starting
//        if (conePlacementState == 1) {
//            attachmentActions.turnTableEncoders(0, true);
//            if (attachmentActions.isDone){
//                conePlacementState = 2;
//            }
//        }
//        //Do this at the start of the function
//        if (conePlacementState == 3) {
//            //If the junction is not in the cone of vision, the robot spins until the junction is detected
//            if (attachmentActions.getJunctionDistance() < 10) {
//                inCone = true;
//            } else if (!spinLeft) {
//                attachmentActions.turnTable.setPower(0.1);
//                conePlacementState = 4;
//            } else if (spinLeft) {
//                attachmentActions.turnTable.setPower(-0.1);
//                conePlacementState = 4;
//            }
//        }
//        //If the junction is already in the cone of vision, turn the other way until it isn't or 90 degrees
//        if (attachmentActions.getJunctionDistance() > 10 && inCone) {
//            inCone = false;
//        }
//        if (inCone) {
//            if (spinLeft) {
//                attachmentActions.turnTable.setPower(0.1);
//            } else if (!spinLeft) {
//                attachmentActions.turnTable.setPower(-0.1);
//            }
//        }
//        //Activates when the junction is within 12.5 degrees of the sensor
//        if (attachmentActions.getJunctionDistance() < 10 && conePlacementState == 4) {
//            //Activates once when the junction is first detected
//            if (!firstDetectedBit) {
//                startingPosition = attachmentActions.getTurntablePosition();
//                startTime = System.currentTimeMillis();
//                firstDetectedBit = true;
//            }
//            attachmentActions.turnTableEncoders(startingPosition, true);
//            //Gets the distance to the junction by averaging the distance over 1/2 second
//            if (System.currentTimeMillis() - startTime < 500) {
//                totalDistance += attachmentActions.getJunctionDistance();
//                i++;
//            } else {
//                distance = totalDistance / i;
//                conePlacementState = 5;
//            }
//        }
//        //If the junction is less than 6.25 inches away, it can use the turntable and extending the grabber, for more accuracy
//        if (attachmentActions.finalDistanceToJunction(distance) < extenderRange && conePlacementState == 5) {
//            usingExtender = true;
//            attachmentActions.turnTableEncoders(attachmentActions.angleToJunction(distance) + startingPosition, true);
//            if (attachmentActions.isDone){
//                conePlacementState = 6;}
//        }
//        //After getting the distance to the junction, it turns towards it
//        if (conePlacementState == 5 && !usingExtender) {
//            attachmentActions.turnTableEncoders(0, true);
//            if (gyroActions.maintainHeading(0.2, startingPosition + attachmentActions.angleToJunction(distance)) && attachmentActions.isDone) {
//                conePlacementState = 6;
//            }
//        }
//        //After turning towards the junction, it lifts the cone to the top of the junction
//        if (conePlacementState == 6) {
//            if (level == LOW) {attachmentActions.setLiftLevel(true, false, false);}
//            if (level == MEDIUM) {attachmentActions.setLiftLevel(false, true, false);}
//            if (level == HIGH) {attachmentActions.setLiftLevel(false, false, true);}
//            //After lifting the cone to the top of the junction, it moves over to the junction
//            if (!attachmentActions.scissorLift1.isBusy()) {
//                if (usingExtender) {
//                    if (!extendingBit) {
//                        extendingTime = System.currentTimeMillis();
//                        extendingBit = true;
//                    }
//                    attachmentActions.extendGripper(attachmentActions.finalDistanceToJunction(distance));
//                    if (((System.currentTimeMillis() - extendingTime) * attachmentActions.finalDistanceToJunction(distance) * extendingSpeed) > 500) {
//                        conePlacementState = 7;
//                    }
//                } else {
//                    encoderActions.encoderDriveNoTimer(200, attachmentActions.finalDistanceToJunction(distance));
//                    if (!encoderActions.motorFrontL.isBusy()) {
//                        conePlacementState = 7;
//                    }
//                }
//            }
//        }
//        //After moving over the junction, it drops the cone
//        if (conePlacementState == 7) {
//            if (!releasingBit) {
//                attachmentActions.openGripper();
//                releasingTime = System.currentTimeMillis();
//            }
//            if (System.currentTimeMillis() - releasingTime > 500){
//                conePlacementState = 8;
//            }
//        }
//        //After dropping the cone, it lowers the lift and finishes the program
//        if (conePlacementState == 8) {
//            if (usingExtender) {
//                attachmentActions.extendGripper(0);
//            }
//            attachmentActions.liftScissor(1000, 0, true);
//            encoderActions.resetEncoder();
//            encoderActions.runWithoutEncoder();
//            initConePlacement();
//        } else {
//            isPlacingCone = true;
//        }
//    }

    private double minSpeed = 0;
    private double distAtMinSpeed = 0;
    private int ticksAtMinSpeed = 0;
    private int counter = 0;
    private boolean coneFinishState = false;
    private double coneDistanceFinal;
    public boolean driveToCone(GyroActions gyroActions, DistanceSensorActions s1, EncoderActions encoderActions, AttachmentActions attachmentActions, double offset, int direction) {
        if (attachmentActions.scissorLift1.getCurrentPosition() < -300 && Math.abs(attachmentActions.getTurntablePosition() - attachmentActions.getTurntableGoal()) < 2) { //If it is possible to see the cone
            if (!coneFinishState) {
                double setSpeed = s1.driveToObject(offset, 700, 400); //Set speed between 700 and 400 as decided by the distance left; it ramps down speed until it gets there
                if (setSpeed > minSpeed) {
                    counter++; //If the speed is more than it was before, the distance has increased, which implies that the cone is leaving the range of the distance sensor
                } else {
                    counter = 0; //Record some measurements for in case it stops seeing the cone
                    minSpeed = setSpeed;
                    distAtMinSpeed = s1.latestDistance;
                    ticksAtMinSpeed = encoderActions.motorFrontL.getCurrentPosition();
                }
                if (counter >= 3) { //If it stops seeing the cone, it goes the rest of the way until it gets to where it thought the cones were when it was seeing the cone
                    coneDistanceFinal = distAtMinSpeed - Math.abs(encoderActions.motorFrontL.getCurrentPosition() - ticksAtMinSpeed);
                    coneFinishState = true;
                } else if (setSpeed == 0) { //If it gets too close to the cones, it uses its last reading to go to the cone
                    coneDistanceFinal = s1.getSensorDistance() - offset;
                    coneFinishState = true;
                } else { //If it's still going to a cone that it can see, it goes at the speed set above by the ramp
                    gyroActions.setVelocityStraight(setSpeed, 0, RIGHT);
                }
            } else { //When it's close to the cone or can't see it, it goes the rest of the way, using encoders and gyros
                if (direction == FORWARDS) {
                    if (!gyroActions.encoderGyroDriveStateMachine(400, coneDistanceFinal, 0)) {
                        return false; //If it's done, it returns false
                    }
                } else if (direction == BACKWARDS) {
                    if (!gyroActions.encoderGyroDriveStateMachine(400, -coneDistanceFinal, 0)) {
                        return false;
                    }
                } else if (direction == RIGHT) {
                    if (!gyroActions.encoderGyroStrafeStateMachine(400, coneDistanceFinal, 0, false)) {
                        return false;
                    }
                } else if (direction == LEFT) {
                    if (!gyroActions.encoderGyroStrafeStateMachine(400, coneDistanceFinal, 0, true)) {
                        return false;
                    }
                }
            }
        } else { //If it is too much off to be able to see the cones, it just keeps adjusting and going forward slowly.
            //TODO: make it go by distance until it starts working, then do the above. ie gyroActions.encoderGyroDrive, but without having to mess around with resetting position
            gyroActions.setVelocityStraight(100, 0, direction);
        }
        return true; //If it's still going, it returns true
    }
    private void initConePlacement() {
        conePlacementState = 0;
        inCone = false;
        startingPosition = 0;
        firstDetectedBit = false;
        startTime = 0;
        totalDistance = 0;
        i = 0;
        distance = 0;
        usingExtender = false;
        extendingBit = false;
        extendingTime = 0;
        extendingSpeed = 200; //Milliseconds per inch, needs to be timed
        releasingBit = false;
        releasingTime = 0;
        isPlacingCone = false;
    }
    public void dudeYouShouldChill (DriveActions driveActions, AttachmentActions attachmentActions) {
        if (Math.abs(attachmentActions.scissorLift1.getCurrentPosition()) > chillThreshold && upBit == false) {
            changeSpeed(driveActions, false, false, false, true);
            upBit = true;
            downBit = false;
        } else if (Math.abs(attachmentActions.scissorLift1.getCurrentPosition()) < chillThreshold && downBit == false) {
            changeSpeed(driveActions, false, false, true, false);
            downBit = true;
            upBit = false;
        }
    }
}