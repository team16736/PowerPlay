package org.firstinspires.ftc.teamcode.actions.distancecalcs;

public class GeometryActions {
    public double distance;
    public double angle;
    double verticalOffset = 8.5625; //Front/back distance from sensor to center of rotation
    double radius = 13; //Distance from center of rotation to center of cone
    public GeometryActions(double distance, double angle){
        this.distance = distance;
        this.angle = angle;
    }
    public double getXFromDistanceAndAngle(){
        return ((distance + verticalOffset) * Math.sin(Math.toRadians(angle)));
    }
    public double getYFromDistanceAndAngle(){
        double x = getXFromDistanceAndAngle();
        return Math.sqrt(Math.pow(distance + verticalOffset, 2) - Math.pow(x, 2)) - radius;
    }
    /*public float s1;
    public float s2;
    public float base;
    public float turntableAngle;
    public float verticalOffset = (float) 8.5625; //Front/back distance from sensor to center of rotation
    public float radius = (float) 13; //Distance from center of rotation to center of cone
    public GeometryActions(float s1, float s2, float base, float turntableAngle) {
        this.s1 = s1;
        this.s2 = s2;
        this.base = base;
        this.turntableAngle = turntableAngle;
    }
    public float calcArea() {
        double s = (s1 + s2 + base) / 2;
        return (float) Math.sqrt(s * (s - s1) * (s - s2) * (s - base));
    }
    public float calcHeight() {
        double area = calcArea();
        return (float) (2 * area) / base;
    }
    public float s1LengthBasePart(){
        double height = calcHeight();
        double sign = 1;
        if(Math.pow(base, 2) + Math.pow(s1, 2) < Math.pow(s2, 2)) {sign = -1;}
        return (float) (Math.sqrt(Math.pow(s1, 2) - Math.pow(height, 2)) * sign);
    }
    public float s2LengthBasePart(){
        double height = calcHeight();
        double sign = 1;
        if(Math.pow(base, 2) + Math.pow(s2, 2) < Math.pow(s1, 2)) {sign = -1;}
        return (float) (Math.sqrt(Math.pow(s2, 2) - Math.pow(height, 2)) * sign);
    }
    public float distanceX(){
        double baseLength = s1LengthBasePart();
        return (float) baseLength - (base / 2);
    }
    public float centerToJunctionY(){
        return (float) calcHeight() + verticalOffset;
    }
    public float angleToJunction(){
        double X = distanceX();
        double Y = centerToJunctionY();
        return (float) Math.toDegrees(Math.atan(X / Y));
    }
    public float centerToJunction(){
        double X = distanceX();
        double Y = centerToJunctionY();
        return (float) Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
    }
    public float absoluteAngle(){
        return (float) (angleToJunction() + turntableAngle);
    }
    public float absoluteX(){
        float angle = absoluteAngle();
        float distance = centerToJunction();
        return (float) Math.sin(Math.toRadians(angle)) * distance;
    }
    public float absoluteY(){
        float angle = absoluteAngle();
        float distance = centerToJunction();
        return (float) Math.cos(Math.toRadians(angle)) * distance;
    }
    public float turntableToX(){
        float X = absoluteX();
        return (float) Math.toDegrees(Math.asin(X / radius));
    }
    public float driveY(){
        float X = absoluteX();
        float Y = (float) Math.sqrt(Math.pow(radius, 2) - Math.pow(X, 2));
        return absoluteY() - Y;
    }*/
}
