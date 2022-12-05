package test.actions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FakeHardware.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.FakeHardware.fakes.drive.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.FakeHardware.fakes.sensors.FakeDistanceSensor;
import org.firstinspires.ftc.teamcode.FakeHardware.fakes.util.FakeHardwareMapFactory;
import org.firstinspires.ftc.teamcode.actions.DriveToWall;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

public class DriveToWallTest {
    HardwareMap fakeHwMap;
    FakeTelemetry fakeTelemetry;
    DriveToWall driveToWall;
    FakeDistanceSensor fakeDistanceSensor;
    FakeDcMotorEx  fakeMotorFrontL;
    FakeDcMotorEx fakeMotorFrontR;
    @Before
    public void setUp() throws IOException, ParserConfigurationException, SAXException {
        fakeHwMap = FakeHardwareMapFactory.getFakeHardwareMap("sample_hardware_map.xml");
        fakeDistanceSensor = new FakeDistanceSensor();
        fakeMotorFrontL = new FakeDcMotorEx();
        fakeMotorFrontR = new FakeDcMotorEx();
        fakeHwMap.put( "distanceSensor", fakeDistanceSensor);
        fakeHwMap.put( "leftFront", fakeMotorFrontL);
        fakeHwMap.put( "rightFront", fakeMotorFrontR);
        fakeTelemetry = new FakeTelemetry();
        driveToWall = new DriveToWall(fakeTelemetry, fakeHwMap);

    }
    @Test
    public void testCloseToObject(){

        fakeDistanceSensor.setDistance(10);
        assert driveToWall.closeToObject(5) == false;
        fakeDistanceSensor.setDistance(10);
        assert driveToWall.closeToObject(15) == true;
    }

    @Test
    public void testMoveIfCloseNotClose(){
        double minDistance = 5;
        double currentDistance = 10;
        fakeDistanceSensor.setDistance(currentDistance);
        boolean notClose = driveToWall.moveIfNotClose(minDistance);
        Assert.assertEquals(false, notClose);
        Assert.assertEquals( 1.0, driveToWall.motorFrontL.getPower(), 0.001);
        Assert.assertEquals( 1.0, driveToWall.motorFrontL.getPower(), 0.001);
    }

    @Test
    public void testMoveIfNotClose(){
        double minDistance = 5;
        double currentDistance = 2;
        fakeDistanceSensor.setDistance(currentDistance);
        boolean notClose = driveToWall.moveIfNotClose(minDistance);
        Assert.assertEquals(true, notClose);
        Assert.assertEquals( 0.0, driveToWall.motorFrontL.getPower(), 0.001);
        Assert.assertEquals( 0.0, driveToWall.motorFrontL.getPower(), 0.001);
    }

}
