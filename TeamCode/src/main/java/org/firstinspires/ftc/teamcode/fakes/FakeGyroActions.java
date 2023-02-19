package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.GyroActions;

public class FakeGyroActions extends GyroActions {
    public FakeGyroActions(LinearOpMode opMode, Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        super(opMode, opModeTelemetry, opModeHardware);
    }
}
