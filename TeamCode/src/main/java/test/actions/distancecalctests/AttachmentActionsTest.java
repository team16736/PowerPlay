package test.actions.distancecalctests;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.AttachmentActions;
import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeCRServo;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.sensors.FakeDistanceSensor;
import org.firstinspires.ftc.teamcode.fakes.util.FakeHardwareMapFactory;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

public class AttachmentActionsTest {
    HardwareMap fakeHwMap;
    FakeTelemetry fakeTelemetry;
    AttachmentActions attachmentActions;

    FakeServo gripperServo;
    FakeServo armExtender;
    FakeCRServo turnTable;
    FakeDcMotorEx scissorLift1;
    FakeDcMotorEx scissorLift2;
    FakeDcMotorEx tableEncoder;

    @Before
    public void setUp() throws IOException, ParserConfigurationException, SAXException {
        fakeHwMap = FakeHardwareMapFactory.getFakeHardwareMap("sample_hardware_map.xml");

        gripperServo = new FakeServo();
        armExtender = new FakeServo();
        turnTable = new FakeCRServo();
        scissorLift1 = new FakeDcMotorEx();
        scissorLift2 = new FakeDcMotorEx();
        tableEncoder = new FakeDcMotorEx();

        fakeHwMap.put(ConfigConstants.GRIPPER_SERVO, gripperServo);
        fakeHwMap.put(ConfigConstants.ARM_EXTENDER, armExtender);
        fakeHwMap.put(ConfigConstants.TURN_TABLE, turnTable);
        fakeHwMap.put(ConfigConstants.SCISSOR_ONE, scissorLift1);
        fakeHwMap.put(ConfigConstants.SCISSOR_TWO, scissorLift2);
        fakeHwMap.put(ConfigConstants.TURN_TABLE_ENCODER, tableEncoder);
        fakeTelemetry = new FakeTelemetry();
        attachmentActions = new AttachmentActions(fakeTelemetry, fakeHwMap);
    }

    @Test
    public void testGetAdjustedDistance() {
        scissorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorLift1.setCurrentPosition(-100);
        attachmentActions.liftToZero();
        Assert.assertEquals(3200, scissorLift1.getVelocity(), 0);
        scissorLift1.setCurrentPosition(-50);
        attachmentActions.liftToZero();
        Assert.assertEquals(-50, attachmentActions.posf, 0);
        Assert.assertFalse(attachmentActions.startf);
        Assert.assertEquals(-100, attachmentActions.startposf, 0);
        Assert.assertEquals(1700, attachmentActions.speedf, 0.01);
        Assert.assertEquals(1700, scissorLift1.getVelocity(), 0);
    }
}
