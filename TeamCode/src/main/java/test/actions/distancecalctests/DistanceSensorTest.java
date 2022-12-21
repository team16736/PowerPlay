package test.actions.distancecalctests;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
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

public class DistanceSensorTest {
    HardwareMap fakeHwMap;
    FakeTelemetry fakeTelemetry;
    DistanceSensorActions distanceSensorActions;
    FakeDistanceSensor fakeS1;
    FakeDistanceSensor fakeS2;

    @Before
    public void setUp() throws IOException, ParserConfigurationException, SAXException {
        fakeHwMap = FakeHardwareMapFactory.getFakeHardwareMap("sample_hardware_map.xml");
        fakeS1 = new FakeDistanceSensor();
        fakeS2 = new FakeDistanceSensor();
        fakeHwMap.put(ConfigConstants.BASE_RANGE, fakeS1);
        fakeHwMap.put(ConfigConstants.GRABBER_RANGE, fakeS2);
        fakeTelemetry = new FakeTelemetry();
        fakeS1.setDistance(300);
        distanceSensorActions = new DistanceSensorActions(fakeHwMap, 0.1, 10, ConfigConstants.BASE_RANGE);
        fakeS1.setDistance(0);
    }

    @Test
    public void testGetAdjustedDistance() {
        fakeS1.setDistance(20);
        Assert.assertEquals(20, distanceSensorActions.avgDistanceLength, 0);
    }

    @Test
    public void testGetExponentialSmoothedDistance() {
        fakeS1.setDistance(10);
        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
        fakeS1.setDistance(100);
        Assert.assertEquals(19, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
        Assert.assertEquals(27.1, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
        Assert.assertEquals(34.39, distanceSensorActions.getExponentialSmoothedDistance(), 0.001);
    }
}
