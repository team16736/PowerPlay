package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveToWall {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    DistanceSensor distanceSensor;
    public DcMotorEx motorFrontL;
    public DcMotorEx motorFrontR;
    public DriveToWall(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;
        motorFrontL = hardwareMap.get(DcMotorEx.class, "leftFront");
        motorFrontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }
    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean closeToObject(double minDistInches){
        double currDist = getDistance();
        return currDist < minDistInches;
    }
    public boolean moveIfNotClose(double minDistInches){
        boolean isClose = closeToObject(minDistInches);
        if (isClose){
            motorFrontL.setPower(0.0);
            motorFrontR.setPower(0.0);
        } else {
            motorFrontL.setPower(1.0);
            motorFrontR.setPower(1.0);

        }
        return isClose;
    }
}
