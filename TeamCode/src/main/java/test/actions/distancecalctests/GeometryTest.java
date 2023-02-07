package test.actions.distancecalctests;


import org.firstinspires.ftc.teamcode.actions.distancecalcs.GeometryActions;
import org.junit.Assert;
import org.junit.Test;

public class GeometryTest {
    @Test
    public void testX(){
        GeometryActions geometry = new GeometryActions(10, 30);
        Assert.assertEquals(5, geometry.getXFromDistanceAndAngle(), 0.0001);
    }
    @Test
    public void testY(){
        GeometryActions geometry = new GeometryActions(10, 30);
        Assert.assertEquals(Math.sqrt(75), geometry.getYFromDistanceAndAngle(), 0.0001);
    }
    /*public void testTriangleArea(){
        GeometryActions geometry = new GeometryActions(4, 6, 3, 0);
        Assert.assertEquals ((float) Math.sqrt(28.4375), geometry.calcArea(), 0);
    }
    @Test
    public void testTriangleHeight(){
        GeometryActions geometry = new GeometryActions(4, 6, 3, 0);
        Assert.assertEquals((float) (Math.sqrt(28.4375) * 2 / 3), geometry.calcHeight(), 0);
    }
    @Test
    public void testS1LengthBasePart(){
        GeometryActions geometry = new GeometryActions(4, 6, 3, 0);
        Assert.assertEquals((float) (-Math.sqrt(16 - Math.pow(Math.sqrt(28.4375) * 2 / 3, 2))), geometry.s1LengthBasePart(), 0.0001);
    }
    @Test
    public void testS2LengthBasePart(){
        GeometryActions geometry = new GeometryActions(4, 6, 3, 0);
        Assert.assertEquals((float) (Math.sqrt(36 - Math.pow(Math.sqrt(28.4375) * 2 / 3, 2))), geometry.s2LengthBasePart(), 0.000);
    }
    @Test
    public void testDistanceX(){
        GeometryActions geometry = new GeometryActions(4, 6, 3, 0);
        Assert.assertEquals ((float) Math.sqrt(28.4375), geometry.calcArea(), 0);
        Assert.assertEquals((float) (Math.sqrt(28.4375) * 2 / 3), geometry.calcHeight(), 0);
        float length = (float) (-Math.sqrt(16 - Math.pow(Math.sqrt(28.4375) * 2 / 3, 2)));
        Assert.assertEquals(length, geometry.s1LengthBasePart(), 0.0001);
        Assert.assertEquals((float) (length - (1.5)), geometry.distanceX(), 0.0001);
    }
    @Test
    public void testDistanceXSimple(){
        GeometryActions geometry = new GeometryActions(4, 4, 5, 0);
        Assert.assertEquals((float) 0, geometry.distanceX(), 0.0001);
    }
    @Test
    public void testDistanceX345(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 0);
        Assert.assertEquals((float) -1.5, geometry.distanceX(), 0.0001);
    }
    @Test
    public void testConeToJunctionY(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 0);
        Assert.assertEquals((float) 4 + 4, geometry.centerToJunctionY(), 0.0001);
    }
    @Test
    public void testAngleToJunction(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 0);
        Assert.assertEquals(Math.toDegrees(Math.atan(-0.1875)), geometry.angleToJunction(), 0.0001);
    }
    @Test
    public void testCenterToJunction(){
        GeometryActions geometry = new GeometryActions(5, 4, 3, 0);
        Assert.assertEquals((float) 8.1394102980498531936765079549919, geometry.centerToJunction(), 0.0001);
    }
    @Test
    public void testAbsoluteAngle(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals((float) Math.toDegrees(Math.atan(-0.1875)) + 30, geometry.absoluteAngle(), 0.0001);
    }
    @Test
    public void testAbsoluteX(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals((float) Math.sin(Math.atan(-0.1875) + Math.toRadians(30)) * 8.1394102980498531936765079549919, geometry.absoluteX(), 0.0001);
    }
    @Test
    public void testAbsoluteY(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals((float) Math.cos(Math.atan(-0.1875) + Math.toRadians(30)) * 8.1394102980498531936765079549919, geometry.absoluteY(), 0.0001);
    }
    @Test
    public void testAll(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals ((float) 6, geometry.calcArea(), 0.0001);
        Assert.assertEquals((float) 4, geometry.calcHeight(), 0.0001);
        Assert.assertEquals((float) 0, geometry.s1LengthBasePart(), 0.0001);
        Assert.assertEquals((float) -1.5, geometry.distanceX(), 0.0001);
        Assert.assertEquals((float) 8, geometry.centerToJunctionY(), 0.0001);
        Assert.assertEquals((float) -10.619655276155134553914504197192, geometry.angleToJunction(), 0.0001);
        Assert.assertEquals((float) 8.1394102980498531936765079549919, geometry.centerToJunction(), 0.0001);
        Assert.assertEquals((float) 19.380344723844865446085495802808, geometry.absoluteAngle(), 0.0001);
        Assert.assertEquals((float) 2.7009618943233420298544152438706, geometry.absoluteX(), 0.0001);
        Assert.assertEquals((float) 7.6782032302755091741097853660235, geometry.absoluteY(), 0.0001);
    }
    @Test
    public void testTurntableToX(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals((float) Math.toDegrees(Math.asin(2.7009618943233420298544152438706 / 6)), geometry.turntableToX(), 0.0001);
    }
    @Test
    public void testDriveY(){
        GeometryActions geometry = new GeometryActions(4, 5, 3, 30);
        Assert.assertEquals((float) 2.32051658630711, geometry.driveY(), 0.0001);
    }*/
}
