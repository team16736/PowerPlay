package org.firstinspires.ftc.teamcode.actions.distancecalcs;

public class GeometryActions {
    public double[] output;
    public double minDistance = 300;
    public int counter = 0;
    public int minCounter = 1;
    public int state = 0;
    public int ticksAtDist = 0;
    public double dist = 8190;
    //    public double TTatDist = 0;
    public int idCounter = 0;
    public int prevTicksBefore = 0;
    public double prevDistBefore = 8190;
    //    public double prevTTBefore = 0;
    public int prevTicksAfter = 0;
    public double prevDistAfter = 8190;
    //    public double prevTTAfter = 0;
    public double[] distanceAtCount;
    public int[] ticksAtCount;
    //    public double[] TTAtCount;
    public boolean checkForError = true;
    public boolean done = false;
    public double[] getDist(double distance, int ticks, boolean detect) {
        if (state == 0) {
            resetGetDist();
            state = 1;
        }
        if (!done) {
            if (distance > minDistance && idCounter > 2) {
                counter = minCounter + 1;
                checkForError = false;
            }
            if (distance > 2000) {
                if (idCounter <= 2) {
                    dist = prevDistBefore;
                    ticksAtDist = prevTicksBefore;
//                    TTatDist = prevTTBefore;
                }
                idCounter = 0;
            }

            if (detect && distance < minDistance) {
                if (idCounter == 0) {
                    prevDistBefore = dist;
                    prevTicksBefore = ticksAtDist;
//                    prevTTBefore = TTatDist;
                }
                idCounter++;

                if (distance < dist) {
                    prevDistAfter = dist;
                    prevTicksAfter = ticksAtDist;
//                    prevTTAfter = TTatDist;
                    dist = distance;
                    ticksAtDist = ticks;
//                    TTatDist = TTPosition;
                }

                if (distance > dist) {
                    distanceAtCount[counter] = distance;
                    ticksAtCount[counter] = ticks;
//                    TTAtCount[counter] = TTPosition;
                    counter++;
                } else {
                    counter = 0;
                }
            }
            if (counter > minCounter) {
                if (checkForError && isError(dist, ticksAtDist, distanceAtCount, ticksAtCount)) {
                    int lowest = getLowest(distanceAtCount, prevDistAfter);
                    if (lowest < 0) {
                        dist = prevDistAfter;
                        ticksAtDist = prevTicksAfter;
//                        TTatDist = prevTTAfter;
                    } else {
                        dist = distanceAtCount[lowest];
                        ticksAtDist = ticksAtCount[lowest];
//                        TTatDist = TTAtCount[lowest];
                    }
                    counter = 0;
                    checkForError = false;
                } else {
                    output[0] = dist;
                    output[1] = (double) ticksAtDist;
//                    output[2] = TTatDist;
                    done = true;
                    state = 0;
//                    RobotLog.dd("FindJunction", "Dist: %f, %f, %f", dist, distanceAtCount[0], distanceAtCount[1]);
//                    RobotLog.dd("FindJunction", "Ticks: %d, %d, %d", ticksAtDist, ticksAtCount[0], ticksAtCount[1]);
                }
            } else {
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            }
        }
        return output;
    }

    public boolean isError(double dist, double distTicks, double[] distanceAtCount, int[] ticksAtCount) {
        boolean error = false;
        if (distanceAtCount[0] > dist) {
            for (int i = 1; i < minCounter + 1; i++) {
                if (distanceAtCount[i] < distanceAtCount[i-1]) {
                    error = true;
//                    RobotLog.dd("FindJunction", "error %d", 1);
                }
            }
        } else {
            error = true;
        }
        return error;
    }

    public int getLowest(double[] distanceAtCount, double dist) {
        double lowest = distanceAtCount[0];
        int lowesti = 0;
        for (int i = 1; i < minCounter + 1; i++) {
            if (distanceAtCount[i] < lowest) {
                lowest = distanceAtCount[i];
                lowesti = i;
            }
        }
        if (dist < lowest) {
            lowest = dist;
            lowesti = -1;
            counter = minCounter + 1;
        } else {
            counter =  minCounter - lowesti;
        }
        return lowesti;
    }

    public void resetGetDist() {
        output = new double[3];
        distanceAtCount = new double[minCounter + 1];
        ticksAtCount = new int[minCounter + 1];
//        TTAtCount = new double[minCounter + 1];
        checkForError = true;
        counter = 0;
        ticksAtDist = 0;
        dist = 8190;
//        TTatDist = 0;
        idCounter = 0;
        prevTicksBefore = 0;
        prevDistBefore = 8190;
//        prevTTBefore = 0;
        prevTicksAfter = 0;
        prevDistAfter = 8190;
//        prevTTAfter = 0;
        done = false;
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
