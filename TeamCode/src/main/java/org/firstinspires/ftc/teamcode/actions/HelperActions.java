package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private double speed = 0.8;
    private int speedingArm = 0;
    private double speedArm = 0.6;

    //stuff for placing cone
    private boolean initConePlacementBit = false;
    private boolean startCenteredBit = false;
    private boolean spinBit;
    private boolean inCone;
    private double startingPosition;
    private boolean firstDetectedBit;
    private double startTime;
    private double totalDistance;
    private int i;
    private boolean averageBit;
    private double distance;
    private boolean usingExtender;
    private boolean atFinalAngle;
    private boolean extendingBit;
    private double extendingTime;
    private double extendingSpeed; //Milliseconds per inch, needs to be timed
    private boolean isInPlace;
    private boolean releasingBit;
    private double releasingTime;
    private boolean isPlaced;
    public boolean isPlacingCone;

    public void drive_ForwardAndStop(DriveActions driveActions, double speed, double drivingTime) {
        driveActions.setMotorDirection_Forward();
        driveActions.driveByTime(this, speed, drivingTime);
        driveActions.stop();
    }

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
        encoderActions.velocity(speed, speed, speed, speed);

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
    public void placeConeOnJunction(AttachmentActions attachmentActions, GyroActions gyroActions, EncoderActions encoderActions, boolean spinLeft, int level) {
        //Initializes the variables the first time the method is called
        if (!initConePlacementBit) {
            initConePlacement();
            initConePlacementBit = true;
        }
        //Center the turntable before starting
        if (!startCenteredBit) {
            attachmentActions.turnTableEncoders(0, 200);
            startCenteredBit = attachmentActions.isDone;
        }
        //Do this at the start of the function
        if (!spinBit && startCenteredBit) {
            //If the junction is not in the cone of vision, the robot spins until the junction is detected
            if (attachmentActions.getJunctionDistance() < 10) {
                inCone = true;
            } else if (!spinLeft) {
                attachmentActions.turnTable.setPower(0.1);
            } else if (spinLeft) {
                attachmentActions.turnTable.setPower(-0.1);
            }
            spinBit = true;
        }
        //If the junction is already in the cone of vision, turn the other way until it isn't or 90 degrees
        if (attachmentActions.getJunctionDistance() > 10 && inCone) {
            inCone = false;
            spinBit = false;
        }
        if (inCone) {
            if (spinLeft) {
                attachmentActions.turnTable.setPower(0.1);
            } else if (!spinLeft) {
                attachmentActions.turnTable.setPower(-0.1);
            }
        }
        //Activates when the junction is within 12.5 degrees of the sensor
        if (attachmentActions.getJunctionDistance() < 10 && !averageBit && !inCone) {
            //Activates once when the junction is first detected
            if (!firstDetectedBit) {
                startingPosition = attachmentActions.getTurntablePosition();
                startTime = System.currentTimeMillis();
                firstDetectedBit = true;
            }
            attachmentActions.turnTableEncoders(startingPosition, 0.2);
            //Gets the distance to the junction by averaging the distance over 1/2 second
            if (System.currentTimeMillis() - startTime < 500) {
                totalDistance += attachmentActions.getJunctionDistance();
                i++;
            } else if (!averageBit) {
                distance = totalDistance / i;
                averageBit = true;
            }
        }
        //If the junction is less than 6.25 inches away, it can use the turntable and extending the grabber, for more accuracy
        //Wait until the turntable is working
        if (attachmentActions.finalDistanceToJunction(distance) < 6.25 && averageBit && !atFinalAngle) {
            usingExtender = true;
            attachmentActions.turnTableEncoders(attachmentActions.angleToJunction(distance) + startingPosition, 0.2);
            if (attachmentActions.isDone){atFinalAngle = true;}
        }
        //After getting the distance to the junction, it turns towards it
        if (averageBit && !atFinalAngle && !usingExtender) {
            attachmentActions.turnTableEncoders(0, 0.2);
            atFinalAngle = gyroActions.maintainHeading(0.2, startingPosition + attachmentActions.angleToJunction(distance)) && attachmentActions.isDone;
        }
        //After turning towards the junction, it lifts the cone to the top of the junction
        if (atFinalAngle && !isInPlace) {
            if (level == LOW) {attachmentActions.setLiftLevel(true, false, false);}
            if (level == MEDIUM) {attachmentActions.setLiftLevel(false, true, false);}
            if (level == HIGH) {attachmentActions.setLiftLevel(false, false, true);}
            //After lifting the cone to the top of the junction, it moves over to the junction
            if (!attachmentActions.scissorLift1.isBusy()) {
                if (usingExtender) {
                    if (!extendingBit) {
                        extendingTime = System.currentTimeMillis();
                        extendingBit = true;
                    }
                    attachmentActions.extendGripper(attachmentActions.finalDistanceToJunction(distance));
                    isInPlace = ((System.currentTimeMillis() - extendingTime) * attachmentActions.finalDistanceToJunction(distance) * extendingSpeed) > 500;
                } else {
                    encoderActions.encoderDriveNoTimer(200, attachmentActions.finalDistanceToJunction(distance));
                    isInPlace = !encoderActions.motorFrontL.isBusy();
                }
            }
        }
        //After moving over the junction, it drops the cone
        if (isInPlace && !isPlaced) {
            if (!releasingBit) {
                attachmentActions.openGripper();
                releasingTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - releasingTime > 500){
                isPlaced = true;
            }
        }
        //After dropping the cone, it lowers the lift and finishes the program
        if (isPlaced) {
            if (usingExtender) {
                attachmentActions.extendGripper(0);
            }
            attachmentActions.liftScissor(1000, 0, true);
            encoderActions.resetEncoder();
            encoderActions.runWithoutEncoder();
            initConePlacement();
        } else {
            isPlacingCone = true;
        }
    }

    private void initConePlacement() {
        spinBit = false;
        inCone = false;
        startingPosition = 0;
        firstDetectedBit = false;
        startTime = 0;
        totalDistance = 0;
        i = 0;
        averageBit = false;
        distance = 0;
        usingExtender = false;
        atFinalAngle = false;
        extendingBit = false;
        extendingTime = 0;
        extendingSpeed = 200; //Milliseconds per inch, needs to be timed
        isInPlace = false;
        releasingBit = false;
        releasingTime = 0;
        isPlaced = false;
        isPlacingCone = false;
    }
}