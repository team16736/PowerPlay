package test.actions.distancecalctests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.DriveActions;
import org.firstinspires.ftc.teamcode.actions.EncoderActions;
import org.firstinspires.ftc.teamcode.actions.FindJunctionAction;
import org.firstinspires.ftc.teamcode.actions.GyroActions;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.fakes.FakeGyroActions;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeCRServo;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.sensors.FakeBNO055IMU;
import org.firstinspires.ftc.teamcode.fakes.sensors.FakeDistanceSensor;
import org.firstinspires.ftc.teamcode.fakes.util.FakeHardwareMapFactory;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

public class FindJunctionActionTest {
    HardwareMap fakeHwMap;
    FakeTelemetry fakeTelemetry;

    LinearOpMode opMode;
    DriveActions driveActions;
    AttachmentActions attachmentActions;
    DistanceSensorActions s1;
    EncoderActions encoderActions;
    GyroActions gyroActions;
    FindJunctionAction findJunctionAction;
    GeometryActions geometry;

    FakeDistanceSensor fakeS1;
    FakeDistanceSensor fakeS2;

    FakeDcMotorEx leftFront;
    FakeDcMotorEx leftRear;
    FakeDcMotorEx rightFront;
    FakeDcMotorEx rightRear;

    FakeServo gripperServo;
    FakeServo armExtender;
    FakeCRServo turnTable;
    FakeDcMotorEx scissorLift1;
    FakeDcMotorEx scissorLift2;
    FakeDcMotorEx tableEncoder;

    FakeBNO055IMU imu2;

    @Before
    public void setUp() throws IOException, ParserConfigurationException, SAXException {
        fakeHwMap = FakeHardwareMapFactory.getFakeHardwareMap("sample_hardware_map.xml");

        fakeS1 = new FakeDistanceSensor();
        fakeS2 = new FakeDistanceSensor();

        leftFront = new FakeDcMotorEx();
        leftRear = new FakeDcMotorEx();
        rightFront = new FakeDcMotorEx();
        rightRear = new FakeDcMotorEx();

        gripperServo = new FakeServo();
        armExtender = new FakeServo();
        turnTable = new FakeCRServo();
        scissorLift1 = new FakeDcMotorEx();
        scissorLift2 = new FakeDcMotorEx();
        tableEncoder = new FakeDcMotorEx();

        imu2 = new FakeBNO055IMU();

        fakeHwMap.put(ConfigConstants.BASE_RANGE, fakeS1);
        fakeHwMap.put(ConfigConstants.GRABBER_RANGE, fakeS2);

        fakeHwMap.put(ConfigConstants.FRONT_LEFT, leftFront);
        fakeHwMap.put(ConfigConstants.BACK_LEFT, leftRear);
        fakeHwMap.put(ConfigConstants.FRONT_RIGHT, rightFront);
        fakeHwMap.put(ConfigConstants.BACK_RIGHT, rightRear);

        fakeHwMap.put(ConfigConstants.GRIPPER_SERVO, gripperServo);
        fakeHwMap.put(ConfigConstants.ARM_EXTENDER, armExtender);
        fakeHwMap.put(ConfigConstants.TURN_TABLE, turnTable);
        fakeHwMap.put(ConfigConstants.SCISSOR_ONE, scissorLift1);
        fakeHwMap.put(ConfigConstants.SCISSOR_TWO, scissorLift2);
        fakeHwMap.put(ConfigConstants.TURN_TABLE_ENCODER, tableEncoder);

        fakeTelemetry = new FakeTelemetry();

        driveActions = new DriveActions(fakeTelemetry, fakeHwMap);
        attachmentActions = new AttachmentActions(fakeTelemetry, fakeHwMap);
        s1 = new DistanceSensorActions(fakeHwMap, 0.5, 10, ConfigConstants.BASE_RANGE);
        encoderActions = new EncoderActions(opMode, fakeTelemetry, fakeHwMap);
        gyroActions = new GyroActions(opMode, fakeTelemetry, fakeHwMap);
        geometry = new GeometryActions();
        findJunctionAction = new FindJunctionAction(fakeHwMap, fakeTelemetry, opMode, driveActions, attachmentActions, s1, encoderActions, gyroActions, geometry);
    }

    @Test
    public void testFakeGyro() {
        imu2.setAngularOrientation(100);
        Orientation orientation = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Assert.assertEquals(100, orientation.firstAngle, 0.001);
    }
}
