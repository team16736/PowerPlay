package org.firstinspires.ftc.teamcode.actions.distancecalcs;

public class GeometryActions {
    public double[] output;
    public double minDistance = 300;
    public int counter = 0; // The number of times it has sensed a distance above the minimum distance it's sensed
    public int minCounter = 1; // If the robot has sensed above its minimum distance is more than this number (if counter is more than mincounter), it triggers the end state
    public int state = 0;
    public int ticksAtDist = 0; // Ticks at lowest distance
    public double dist = 8190; // Lowest distance detected
    //    public double TTatDist = 0;
    public int idCounter = 0; // Number of times in a row the robot has seen the pole
    public int prevTicksBefore = 0; // Previous ticks to reset to when the robot detects a randomly low sensor distance before seeing the actual pole
    public double prevDistBefore = 8190; // Same as prevTicksBefore but with sensor distance
    //    public double prevTTBefore = 0;
    public int prevTicksAfter = 0; // Previous ticks to reset to when the robot detects a random low sensor distance while seeing the actual pole
    public double prevDistAfter = 8190; // Same as prevTicksAfter but with sensor distance
    //    public double prevTTAfter = 0;
    public double[] distanceAtCount; // The sensor distance detected for the first couple of detections that are higher than the minimum distance
    public int[] ticksAtCount; // Same as distanceAtCount but with ticks
    //    public double[] TTAtCount;
    public boolean checkForError = true; // Whether or not it will check whether the current minimum is an error
    public boolean done = false; // Whether it knows it has sensed the junction
    public double[] getDist(double distance, int ticks, boolean detect) { //When it has confirmed detection of the pole, returns the ticks and sensor distance at the lowest point
        if (state == 0) {
            resetGetDist();
            state = 1;
        }
        if (!done) {
            if (distance > minDistance && idCounter > 2) { //If the sensor has detected the pole several times in a row and then not detected the pole, it has finished finding the pole
                counter = minCounter + 1;
                checkForError = false;
            }
            if (distance > 2000) { // If it isn't sensing anything; usually at 8190 or 8191
                if (idCounter <= 2) { // If it has seen one or two low distances in a row and then a high one, it's an error and discarded
                    dist = prevDistBefore;
                    ticksAtDist = prevTicksBefore;
//                    TTatDist = prevTTBefore;
                }
                idCounter = 0; // Resetting idCounter so that it tells us how many detections in a row, not total detections
            }

            if (detect && distance < minDistance) { // If it's seeing the junction and all of the other things are out of the way
                if (idCounter == 0) { // If it's the first time seeing the junction in a row, record the previous minimum distance in case the detection was a glitch
                    prevDistBefore = dist;
                    prevTicksBefore = ticksAtDist;
//                    prevTTBefore = TTatDist;
                }
                idCounter++; // idCounter counts how many times it's seen the junction

                if (distance < dist) { // If it's detecting the lowest distance it's seen, it records the previous minimum distance in case this one's a glitch, then records the current distance and ticks in case it's the global minimum
                    prevDistAfter = dist;
                    prevTicksAfter = ticksAtDist;
//                    prevTTAfter = TTatDist;
                    dist = distance;
                    ticksAtDist = ticks;
//                    TTatDist = TTPosition;
                }

                if (distance > dist) { // If it's detecting more than the minimum it's detected, it records the current position and distance it's detected, as well as how long it's been since it detected the minimum value. For error detection
                    distanceAtCount[counter] = distance;
                    ticksAtCount[counter] = ticks;
//                    TTAtCount[counter] = TTPosition;
                    counter++; // To tell how many times we've detected a distance above the minimum distance detected
                } else { // If the distance is less than the minimum, reset the counter
                    counter = 0;
                }
            }
            if (counter > minCounter) { // If the robot has detected multiple values above the minimum value, it's probably done
                if (checkForError && isError(dist, ticksAtDist, distanceAtCount, ticksAtCount)) { // If it's an error, take the error value out of the data set and keep checking
                    int lowest = getLowest(distanceAtCount, prevDistAfter);
                    if (lowest < 0) { // If the value is below zero, it means the lowest value was what was detected before the glitch minimum value
                        dist = prevDistAfter;
                        ticksAtDist = prevTicksAfter;
//                        TTatDist = prevTTAfter;
                    } else { // If it's above 0, it tells us which of the values gotten after the glitch minimum value was the lowest
                        dist = distanceAtCount[lowest];
                        ticksAtDist = ticksAtCount[lowest];
//                        TTatDist = TTAtCount[lowest];
                    }
                    counter = 0;
                    checkForError = false; // Assuming there's never more than one error detection, if there's been an error we stop checking for another one. This kind of error is very rare, only happens every once in a while, so this is a safe assumption
                } else { // If it isn't an error, it's where the junction is, which we report as such.
                    output[0] = dist;
                    output[1] = (double) ticksAtDist;
//                    output[2] = TTatDist;
                    done = true;
                    state = 0;
//                    RobotLog.dd("FindJunction", "Dist: %f, %f, %f", dist, distanceAtCount[0], distanceAtCount[1]);
//                    RobotLog.dd("FindJunction", "Ticks: %d, %d, %d", ticksAtDist, ticksAtCount[0], ticksAtCount[1]);
                }
            } else { // If we haven't detected the pole, return nothing
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            }
        }
        return output;
    }

    public boolean isError(double dist, double distTicks, double[] distanceAtCount, int[] ticksAtCount) { // Checking if the minimum value is an glitch
        boolean error = false;
        if (distanceAtCount[0] > dist) { // If the distance sensed doesn't continually go up after the minimum value, there was a glitch and we throw out the minimum value and try again
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

    public int getLowest(double[] distanceAtCount, double dist) { // We find which was the lowest distance and then tell which one it was
        double lowest = distanceAtCount[0];
        int lowesti = 0;
        for (int i = 1; i < minCounter + 1; i++) { // If the lowest value is found after detecting the glitch, we tell it which one it was by how long it was from when the minimum was detected
            if (distanceAtCount[i] < lowest) {
                lowest = distanceAtCount[i];
                lowesti = i;
            }
        }
        if (dist < lowest) { // If the lowest value was found before the glitched value, we give it as a -1
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
