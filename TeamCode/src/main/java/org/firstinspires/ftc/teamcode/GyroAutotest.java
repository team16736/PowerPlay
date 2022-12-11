package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.GyroActions;

@Autonomous(name="Gyro Auto", group="Robot")
public class GyroAutotest extends LinearOpMode {
    @Override
    public void runOpMode() {
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = ");
            telemetry.update();
        }

        GyroActions gyroActions = new GyroActions(this, telemetry, hardwareMap);
        //gyroActions.driveStraight(0.4,10.0,0);
        gyroActions.driveStraight(0.2,10.0,90);
    }
}
