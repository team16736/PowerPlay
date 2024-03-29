package org.firstinspires.ftc.teamcode.actions;
// import lines were omitted. OnBotJava will add them automatically.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


final public class EncoderActions{
    public DcMotorEx motorFrontL;
    public DcMotorEx motorFrontR;
    public DcMotorEx motorBackL;
    public DcMotorEx motorBackR;
    private static LinearOpMode opModeObj;


    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();

    public EncoderActions() {

    }

    public EncoderActions(LinearOpMode opMode, Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        opModeObj = opMode;
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;
        motorFrontL = hardwareMap.get(DcMotorEx.class, "leftFront");
        motorFrontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        motorBackL = hardwareMap.get(DcMotorEx.class, "leftRear");
        motorBackR = hardwareMap.get(DcMotorEx.class, "rightRear");
    }
    public void encoderDrive(double encoderSpeed, double encoderDistance) { // Deprecated. Look to GyroActions for auto driving
        resetEncoder();

        double ticksPerInch = 32.2;
        int totalTicks = (int) (ticksPerInch * encoderDistance);
        motorFrontL.setTargetPosition(totalTicks);
        motorFrontR.setTargetPosition(totalTicks);
        motorBackL.setTargetPosition(totalTicks);
        motorBackR.setTargetPosition(totalTicks);


        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setVelocity(encoderSpeed, encoderSpeed, encoderSpeed, encoderSpeed);

        // While the Op Mode is running, show the motor's status via telemetry
        whileMotorBusy();
    }

    public void encoderDriveNoTimer(double encoderSpeed, double encoderDistance) { // Deprecated
        resetEncoder();
        // Set the motor's target position to 6.4 rotations
        double ticksPerInch = 32.2;
        int totalTicks = (int) (ticksPerInch * encoderDistance);
        motorFrontL.setTargetPosition(totalTicks);
        motorFrontR.setTargetPosition(totalTicks);
        motorBackL.setTargetPosition(totalTicks);
        motorBackR.setTargetPosition(totalTicks);


        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 1 revolution per second
        setVelocity(encoderSpeed, encoderSpeed, encoderSpeed, encoderSpeed);
    }
    public void encoderStrafe(double encoderSpeed,
                              double encoderDistance,
                              boolean encoderMoveLeft) { // Deprecated
        encoderStrafeNoWhile(encoderSpeed, encoderDistance, encoderMoveLeft);

        //motorFrontL.isBusy()hile the Op Mode is running, show the motor's status via telemetry
        while (motorFrontL.isBusy()) {
            telemetry.addData("FL is at target", !motorFrontL.isBusy());
            telemetry.addData("FR is at target", !motorFrontR.isBusy());
            telemetry.addData("BL is at target", !motorBackL.isBusy());
            telemetry.addData("BR is at target", !motorBackR.isBusy());
            telemetry.update();
        }
    }
    public void encoderStrafeNoWhile(double encoderSpeed,
                              double encoderDistance,
                              boolean encoderMoveLeft) { // No longer used much
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor's target position to 6.4 rotations
        double ticksPerInch = 33.6;
        int totalTicks = (int) (ticksPerInch * encoderDistance);
        if (encoderMoveLeft){
            motorFrontL.setTargetPosition(-totalTicks);
            motorFrontR.setTargetPosition(totalTicks);
            motorBackL.setTargetPosition(totalTicks);
            motorBackR.setTargetPosition(-totalTicks);
        }else{
            motorFrontL.setTargetPosition(totalTicks);
            motorFrontR.setTargetPosition(-totalTicks);
            motorBackL.setTargetPosition(-totalTicks);
            motorBackR.setTargetPosition(totalTicks);
        }


        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 1 revolution per second
        motorFrontL.setVelocity(-encoderSpeed);
        motorFrontR.setVelocity(-encoderSpeed);
        motorBackL.setVelocity(-encoderSpeed);
        motorBackR.setVelocity(-encoderSpeed);
    }
    public void encoderSpin(double encoderSpeed,
                              double encoderDegrees,
                              boolean encoderSpinLeft) { // Deprecated
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor's target position to 6.4 rotations
        double ticksPerDegree = 5.3;
        int totalTicks = (int) (ticksPerDegree * encoderDegrees);
        if (encoderSpinLeft){
            motorFrontL.setTargetPosition(-totalTicks);
            motorFrontR.setTargetPosition(totalTicks);
            motorBackL.setTargetPosition(-totalTicks);
            motorBackR.setTargetPosition(totalTicks);
        }else{
            motorFrontL.setTargetPosition(totalTicks);
            motorFrontR.setTargetPosition(-totalTicks);
            motorBackL.setTargetPosition(totalTicks);
            motorBackR.setTargetPosition(-totalTicks);
        }


        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 1 revolution per second
        motorFrontL.setVelocity(-encoderSpeed);
        motorFrontR.setVelocity(-encoderSpeed);
        motorBackL.setVelocity(-encoderSpeed);
        motorBackR.setVelocity(-encoderSpeed);

        //motorFrontL.isBusy()hile the Op Mode is running, show the motor's status via telemetry
        while (motorFrontL.isBusy()) {
            telemetry.addData("FL is at target", !motorFrontL.isBusy());
            telemetry.addData("FR is at target", !motorFrontR.isBusy());
            telemetry.addData("BL is at target", !motorBackL.isBusy());
            telemetry.addData("BR is at target", !motorBackR.isBusy());
            telemetry.update();
        }
    }

    public void encoderSpinNoWhile(double encoderSpeed,
                            double encoderDegrees,
                            boolean encoderSpinLeft) { // Deprecated
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor's target position to 6.4 rotations
        double ticksPerDegree = 5.393;
        int totalTicks = (int) (ticksPerDegree * encoderDegrees);
        if (encoderSpinLeft){
            motorFrontL.setTargetPosition(-totalTicks);
            motorFrontR.setTargetPosition(totalTicks);
            motorBackL.setTargetPosition(-totalTicks);
            motorBackR.setTargetPosition(totalTicks);
        }else{
            motorFrontL.setTargetPosition(totalTicks);
            motorFrontR.setTargetPosition(-totalTicks);
            motorBackL.setTargetPosition(totalTicks);
            motorBackR.setTargetPosition(-totalTicks);
        }


        // Switch to RUN_TO_POSITION mode
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setVelocity(-encoderSpeed, -encoderSpeed, -encoderSpeed, -encoderSpeed);
    }

//    public void fancySpin(double speed, double distance, boolean moveLeft){
//        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        // Set the motor's target position to 6.4 rotations
//        double ticksPerInch = 64.75;
//        int totalTicks = (int) (ticksPerInch * distance);
//        if (moveLeft){
//            motorFrontL.setTargetPosition(-totalTicks);
//            motorFrontR.setTargetPosition(totalTicks);
//            motorBackL.setTargetPosition(totalTicks);
//            motorBackR.setTargetPosition(-totalTicks);
//        }else{
//            motorFrontL.setTargetPosition(totalTicks); // 2 7/8 rotation
//            motorFrontR.setTargetPosition(-totalTicks / 4);
//            motorBackL.setTargetPosition(-totalTicks);
//            motorBackR.setTargetPosition(totalTicks * 4);
//        }
//
//
//        // Switch to RUN_TO_POSITION mode
//        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Start the motor moving by setting the max velocity to 1 revolution per second
//        if (moveLeft) {
//            motorFrontL.setVelocity(-speed / 2);
//            motorFrontR.setVelocity(-speed);
//            motorBackL.setVelocity(-speed * 2);
//            motorBackR.setVelocity(-speed);
//        } else {
//            motorFrontL.setVelocity(-speed);
//            motorFrontR.setVelocity(-speed / 4);
//            motorBackL.setVelocity(-speed);
//            motorBackR.setVelocity(-speed * 4);
//        }
//
//        //motorFrontL.isBusy()hile the Op Mode is running, show the motor's status via telemetry
//        while (motorFrontL.isBusy()) {
//            telemetry.addData("FL is at target", !motorFrontL.isBusy());
//            telemetry.addData("FR is at target", !motorFrontR.isBusy());
//            telemetry.addData("BL is at target", !motorBackL.isBusy());
//            telemetry.addData("BR is at target", !motorBackR.isBusy());
//            telemetry.update();
//        }
//    }

    public void whileMotorBusy(){
        while (motorFrontL.isBusy() && motorFrontR.isBusy() && motorBackL.isBusy() && motorBackR.isBusy()) {
            telemetry.addData("FL is at target", !motorFrontL.isBusy());
            telemetry.addData("FR is at target", !motorFrontR.isBusy());
            telemetry.addData("BL is at target", !motorBackL.isBusy());
            telemetry.addData("BR is at target", !motorBackR.isBusy());
            telemetry.update();
        }
    }
    public void setVelocity(double lf, double rf, double lb, double rb){
        motorFrontL.setVelocity(lf);
        motorFrontR.setVelocity(rf);
        motorBackL.setVelocity(lb);
        motorBackR.setVelocity(rb);
    }
    public void resetEncoder(){
        motorFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition() {
        motorFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runWithoutEncoder(){
        motorFrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int getPosition(){
        return motorFrontL.getCurrentPosition();
    }
    public void stopEncoderMotors(){
        int error = motorFrontL.getTargetPosition() - motorFrontL.getCurrentPosition();
        motorFrontL.setTargetPosition(motorFrontL.getTargetPosition() - error);
        motorFrontR.setTargetPosition(motorFrontR.getTargetPosition() - error);
        motorBackL.setTargetPosition(motorBackL.getTargetPosition() - error);
        motorBackR.setTargetPosition(motorBackR.getTargetPosition() - error);
    }
}

