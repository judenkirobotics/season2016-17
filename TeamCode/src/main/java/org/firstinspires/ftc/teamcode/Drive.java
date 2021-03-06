package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by judenki on 11/12/16.
 *
 * NEED TO DO;
 *
 * Add open loop (no time) and closed loop (time based) controls for moveForward and turn
 *
 * Rename moveForward() to move()
 *
 * DONE -- Add method to handle WHEEL_CIRC, WHEEL_RPM, and TURN_PER_SECOND.
 *
 * Get motors to run with the encoder active to utilize the controllers PID.
 */

public class Drive {

    private DcMotor[] leftMotors   = null;
    private DcMotor[] rightMotors  = null;

    private double MIN_DRIVE_DISTANCE = 0.0;
    private double MAX_DRIVE_DISTANCE = 120.0;

    private double FORWARD_POWER =  0.8;
    private double REVERSE_POWER = -0.8;
    private double TURN_POWER    =  0.4;

    private double WHEEL_CIRC    = 13;
    private double WHEEL_RPM     = 2;   //Misnamed, fix should be RPS second not minute

    private double TURN_PER_SECOND = 79.5;

    private long driveStopTime  = 0;

    private boolean motorsStopped = true;


    // Left and right are with respect to the robot
    public Drive( DcMotor[] _leftMotors, DcMotor[] _rightMotors ) {
        assert leftMotors != null;
        assert rightMotors != null;
        assert leftMotors.length > 0;
        assert rightMotors.length > 0;
        this.leftMotors = _leftMotors;
        this.rightMotors = _rightMotors;
        // Set all DC Motors to run without encoders
        for( DcMotor dcm : leftMotors ) {
            dcm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }



    public void turn(double degrees, double power) {
        long time;
        double leftSign;
        double rightSign;

        if (degrees > 0) {
            rightSign = -1;
            leftSign  = -1;
            time = turnTime(degrees);
        }
        else {
            rightSign = 1;
            leftSign  = 1;
            time = turnTime(degrees) * -1;
        }

        for(DcMotor dcm : leftMotors){
            dcm.setPower(leftSign * power);
        }

        for(DcMotor dcm : rightMotors) {
            dcm.setPower(rightSign * power);
        }

        driveStopTime = time + SystemClock.elapsedRealtime();
        motorsStopped = false;
    }


    public void drift(double power) {
        if (power > 0) {
            for(DcMotor dcm : leftMotors){
                dcm.setPower(dcm.getPower() +(power));
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(dcm.getPower() +(power));
            }
        }
        else {
            for(DcMotor dcm : leftMotors){
                dcm.setPower(dcm.getPower() +(power));
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(dcm.getPower() +(power));
            }
        }

    }

    public void driveMove(double forwardPower, double driftPower){
        for (DcMotor dcm : leftMotors){
            dcm.setPower( - forwardPower + driftPower);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( forwardPower + driftPower);
        }

    }


    public void moveForward(double distance , double power) {

        double moveDistance;
        long   time;

        // Input distance will be in inches, perform a range check
        moveDistance = rangeCheck(distance);

        //Calculate how long to move.
        time =  moveTime(moveDistance, WHEEL_RPM, WHEEL_CIRC);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (DcMotor dcm : leftMotors){
            dcm.setPower( - power);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( power);
        }

        //SystemClock.sleep((long)(double)time);
        //SystemClock.sleep((long)(double)time);
        driveStopTime = time + SystemClock.elapsedRealtime();
        motorsStopped = false;

        /*
        for(DcMotor dcm : leftMotors) {
            dcm.setPower(0.0);
        }
        for(DcMotor dcm : rightMotors) {
            dcm.setPower(0.0);
        }
        */

    }


    public void update() {
        if  (SystemClock.elapsedRealtime()>driveStopTime) {
            for(DcMotor dcm : leftMotors) {
                dcm.setPower(0.0);
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(0.0);
            }
            motorsStopped = true;
        }
    }


    public boolean motorsRunning () {
        return (!motorsStopped);
    }

    public void allStop(){
        for(DcMotor dcm : leftMotors) {
            dcm.setPower(0.0);
        }
        for(DcMotor dcm : rightMotors) {
            dcm.setPower(0.0);
        }
        motorsStopped = true;
    }

    public void setParams(double wheelCirc, double wheelRPM, double turnPerSecond) {
        WHEEL_CIRC = wheelCirc;
        WHEEL_RPM = wheelRPM;
        TURN_PER_SECOND = turnPerSecond;
    }


    private double rangeCheck(double distance) {
        if (distance < MIN_DRIVE_DISTANCE)
            return 0;
        if (distance > MAX_DRIVE_DISTANCE)
            return MAX_DRIVE_DISTANCE;
        return distance;
    }

    private long moveTime(double distance, double rpm, double circumferance) {
        return (long)(distance / (rpm * circumferance) * 1000);
    }

    private long turnTime(double degrees) {
        return (long)((degrees / TURN_PER_SECOND) *1000);
    }



}