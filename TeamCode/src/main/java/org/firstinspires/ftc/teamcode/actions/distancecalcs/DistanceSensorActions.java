package org.firstinspires.ftc.teamcode.actions.distancecalcs;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorActions {
    private HardwareMap hardwareMap;

    public int avgDistanceLength;
    public double mostRecentAdjusterAvg = 3;
    private double[] distancesAvg;
    private double sumAvg = 0;
    private double sumAllAvg = 0;
    private DistanceSensor distanceSensor;

    private double exponentialSmoothedDistance = 0;
    public double alpha = 0.1;

    public DistanceSensorActions(HardwareMap opModeHardware, double alpha, int avgDistanceLength, String distanceSensor) {
        this.hardwareMap = opModeHardware;
        this.alpha = alpha;
        this.avgDistanceLength = avgDistanceLength;
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, distanceSensor);

        distancesAvg = new double[avgDistanceLength];
        resetSmoothers();
    }

    public double getSensorDistance(){
        return getSensorDistance(DistanceUnit.INCH);
    }

    public double getSensorDistance(DistanceUnit distanceUnit) {
        return distanceSensor.getDistance(distanceUnit);
    }

    public double getAverageDistanceLive(){
        sumAvg = 0;
        for (int i = 1; i < avgDistanceLength; i++) {
            distancesAvg[(avgDistanceLength) - i] = distancesAvg[(avgDistanceLength - 1) - i];
        }
        distancesAvg[0] = distanceSensor.getDistance(DistanceUnit.INCH);
        for (int i = 1; i < avgDistanceLength; i++) {
            sumAvg += distancesAvg[i];
        }
        sumAvg += distancesAvg[0] * mostRecentAdjusterAvg;
        return sumAvg / (avgDistanceLength + mostRecentAdjusterAvg - 1);
    }

    public double getAverageDistanceAllInOne() {
        sumAllAvg = 0;
        for (int i = 0; i < avgDistanceLength; i++) {
            sumAllAvg += distanceSensor.getDistance(DistanceUnit.INCH);
        }
        return sumAllAvg / avgDistanceLength;
    }

    public double getExponentialSmoothedDistance(){
        if(exponentialSmoothedDistance == 0){
            exponentialSmoothedDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        } else {
            exponentialSmoothedDistance = alpha * distanceSensor.getDistance(DistanceUnit.INCH) + (1 - alpha) * exponentialSmoothedDistance;
        }
        return exponentialSmoothedDistance;
    }

    public void resetSmoothers(){
        exponentialSmoothedDistance = 0;
        for (int i = 0; i < avgDistanceLength; i++) {
            distancesAvg[i] = distanceSensor.getDistance(DistanceUnit.INCH);
        }
    }
}
