//package test.actions.distancecalctests;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.FakeHardware.fakes.FakeTelemetry;
//import org.firstinspires.ftc.teamcode.FakeHardware.fakes.sensors.FakeDistanceSensor;
//import org.firstinspires.ftc.teamcode.FakeHardware.fakes.util.FakeHardwareMapFactory;
//import org.firstinspires.ftc.teamcode.actions.distancecalcs.DistanceSensorActions;
//import org.firstinspires.ftc.teamcode.actions.constants.ConfigConstants;
//import org.junit.Assert;
//import org.junit.Before;
//import org.junit.Test;
//import org.xml.sax.SAXException;
//
//import java.io.IOException;
//
//import javax.xml.parsers.ParserConfigurationException;
//
//public class DistanceSensorTest {
//    HardwareMap fakeHwMap;
//    FakeTelemetry fakeTelemetry;
//    DistanceSensorActions distanceSensorActions;
//    FakeDistanceSensor fakeS1;
//    FakeDistanceSensor fakeS2;
//
//    @Before
//    public void setUp() throws IOException, ParserConfigurationException, SAXException {
//        fakeHwMap = FakeHardwareMapFactory.getFakeHardwareMap("sample_hardware_map.xml");
//        fakeS1 = new FakeDistanceSensor();
//        fakeS2 = new FakeDistanceSensor();
//        fakeHwMap.put(ConfigConstants.S1DISTANCESENSOR, fakeS1);
//        fakeHwMap.put(ConfigConstants.S2DISTANCESENSOR, fakeS2);
//        fakeTelemetry = new FakeTelemetry();
//        fakeS1.setDistance(300);
//        distanceSensorActions = new DistanceSensorActions(fakeHwMap, 0.1, 10);
//        fakeS1.setDistance(0);
//    }
//
//    @Test
//    public void testGetS1() {
//        fakeS1.setDistance(10);
//        assert distanceSensorActions.getS1() == 10;
//    }
//
//    @Test
//    public void testGetS2() {
//        fakeS2.setDistance(10);
//        assert distanceSensorActions.getS2() == 10;
//    }
//
//    @Test
//    public void testReturnLargerSide() {
//        fakeS1.setDistance(11);
//        fakeS2.setDistance(10);
//        assert distanceSensorActions.returnLargerSide() == "s1";
//    }
//
//    @Test
//    public void testGetAdjustedDistance() {
//        fakeS1.setDistance(20);
//        Assert.assertEquals((300 * 9 + 3 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 8 + 4 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 7 + 5 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 6 + 6 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 5 + 7 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 4 + 8 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 3 + 9 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 2 + 10 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 1 + 11 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals((300 * 0 + 12 * fakeS1.getDistance(DistanceUnit.INCH)) / (distanceSensorActions.avgDistanceLength - 1 + distanceSensorActions.mostRecentAdjusterAvg), distanceSensorActions.getAverageDistance(distanceSensorActions.s1), 0.001);
//    }
//
//    @Test
//    public void testGetExponentialSmoothedDistance() {
//        fakeS1.setDistance(10);
//        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals(10, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//        fakeS1.setDistance(100);
//        Assert.assertEquals(19, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals(27.1, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//        Assert.assertEquals(34.39, distanceSensorActions.getExponentialSmoothedDistance(distanceSensorActions.s1), 0.001);
//    }
//}
