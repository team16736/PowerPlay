package org.firstinspires.ftc.teamcode.actions.distancecalcs;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorActions {
    private HardwareMap hardwareMap;

    public int avgDistanceLength;
    public double mostRecentAdjusterAvg = 1;
    private double[] distancesAvg;
    private double sumAvg = 0;
    private double sumAllAvg = 0;
    private double sumAllDistance;
    private DistanceSensor distanceSensor;

    private double exponentialSmoothedAverage = 0;
    private double exponentialSmoothedDistance = 0;
    public double latestDistance;

    public double alpha = 0.1;

    //Drive to Object
    double a;
    public int driveToObjectState = 0;
    double startingDistance = 0;
    double speed;
    double distance;
    public double measureDistance;

    public DistanceSensorActions(HardwareMap opModeHardware, double alpha, int avgDistanceLength, String distanceSensor) {
        this.hardwareMap = opModeHardware;
        this.alpha = alpha;
        this.avgDistanceLength = avgDistanceLength;
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, distanceSensor);

        distancesAvg = new double[avgDistanceLength];
        resetSmoothers();
    }

    public double getSensorDistance(){
        latestDistance = getSensorDistance(DistanceUnit.INCH);
        return latestDistance;
    }

    public double getSensorDistance(DistanceUnit distanceUnit) {
        return distanceSensor.getDistance(distanceUnit);
    }

    public double getAverageDistanceLive(){
        sumAvg = 0;
        for (int i = 1; i < avgDistanceLength; i++) {
            distancesAvg[(avgDistanceLength) - i] = distancesAvg[(avgDistanceLength - 1) - i];
        }
        distancesAvg[0] = getSensorDistance();
        for (int i = 1; i < avgDistanceLength; i++) {
            sumAvg += distancesAvg[i];
        }
        sumAvg += distancesAvg[0] * mostRecentAdjusterAvg;
        return sumAvg / (avgDistanceLength + mostRecentAdjusterAvg - 1);
    }

    public double getAverageDistanceAllInOne(boolean under70) {
        sumAllAvg = 0;
        for (int i = 0; i < avgDistanceLength; i++) { //avgDistanceLength set to 10 during init as of 1/19/2023
            sumAllDistance = getSensorDistance();
            if (under70 && sumAllDistance > 80) {
                for (int j = 0; j < 10; j++) {
                    sumAllDistance = getSensorDistance();
                    if (sumAllDistance < 80) {
                        j = 100;
                    }
                }
                if (sumAllDistance > 80) {
                    sumAllDistance = sumAllAvg / i + 1;
                }
            }
            sumAllAvg += sumAllDistance;
        }
        return sumAllAvg / avgDistanceLength;
    }

    public double getExponentialSmoothedDistance(boolean under70){
        exponentialSmoothedDistance = getSensorDistance();
        if (under70 && exponentialSmoothedDistance > 80) {
            if (exponentialSmoothedAverage == 0) {
                for (int j = 0; j < 10; j++) {
                    exponentialSmoothedDistance = getSensorDistance();
                    if (exponentialSmoothedDistance < 80) {
                        j = 100;
                    }
                }
                if (exponentialSmoothedDistance > 80) {
                    return getSensorDistance();
                }
            }
        } else if(exponentialSmoothedAverage == 0){
            exponentialSmoothedAverage = exponentialSmoothedDistance;
        } else {
            exponentialSmoothedAverage = alpha * exponentialSmoothedDistance + (1 - alpha) * exponentialSmoothedAverage;
        }
        return exponentialSmoothedAverage;
    }

    public double driveToObject(double offset, double maxSpeed, double minSpeed) {
        speed = maxSpeed;
        if (driveToObjectState == 0) {
            startingDistance = getAverageDistanceAllInOne(true) - offset;
            RobotLog.dd("FindJunction", "Distance Left: %f", startingDistance);
            distance = 0;
            driveToObjectState = 1;
        } else {
            measureDistance = getExponentialSmoothedDistance(true);
            distance = startingDistance - (getExponentialSmoothedDistance(true) - offset);
        }
        if (Math.abs(startingDistance - distance) < 2) {
            startingDistance = 0;
            driveToObjectState = 0;
            return 0;
        } else if (distance > startingDistance * 0.25) {
            a = distance - (startingDistance * (0.25));
            speed = a * (minSpeed - maxSpeed) / (0.75 * startingDistance) + maxSpeed;
        }
        return Range.clip(speed, minSpeed, maxSpeed);
    }

    public void resetSmoothers(){
        exponentialSmoothedAverage = 0;
        for (int i = 0; i < avgDistanceLength; i++) {
            distancesAvg[i] = getSensorDistance();
        }
    }
}
