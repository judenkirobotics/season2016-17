package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by judenki on 11/12/16.
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


    public void turn(double degrees) {
        double time;
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
            dcm.setPower(leftSign * TURN_POWER);
        }

        for(DcMotor dcm : rightMotors) {
            dcm.setPower(rightSign * TURN_POWER);
        }

        SystemClock.sleep((long)(double)time);

        for(DcMotor dcm : leftMotors) {
            dcm.setPower(0.0);
        }

        for(DcMotor dcm : rightMotors) {
            dcm.setPower(0.0);
        }


    }

    public void moveForward(double distance) {

        double moveDistance;
        double   time;
        // Input distance will be in inches, perform a range check
        moveDistance = rangeCheck(distance);

        //Calculate how long to move.
        time =  moveTime(moveDistance, WHEEL_RPM, WHEEL_CIRC);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (DcMotor dcm : leftMotors){
            dcm.setPower( - FORWARD_POWER);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( FORWARD_POWER);
        }

        //SystemClock.sleep((long)(double)time);
        SystemClock.sleep((long)(double)time);

        for(DcMotor dcm : leftMotors) {
            dcm.setPower(0.0);
        }
        for(DcMotor dcm : rightMotors) {
            dcm.setPower(0.0);
        }

    }

    /*
    public void moveBackward(double distance) {
        double moveDistance;
        double   time;
        // Input distance will be in inches, perform a range check
        moveDistance = rangeCheck(distance);

        //Calculate how long to move.
        time =  moveTime(moveDistance, WHEEL_RPM, WHEEL_CIRC);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotors.setPower(REVERSE_POWER);
        //leftMotor.setMaxSpeed(100);
        rightMotors.setPower(REVERSE_POWER);
        //rightMotor.setMaxSpeed(100);

        //SystemClock.sleep((long)(double)time);
        SystemClock.sleep((long)(double)time);

        leftMotors.setPower(0.0);
        rightMotors.setPower(0.0);

    }
    */

    public void update() {

    }

    private double rangeCheck(double distance) {
        if (distance < MIN_DRIVE_DISTANCE)
            return 0;
        if (distance > MAX_DRIVE_DISTANCE)
            return MAX_DRIVE_DISTANCE;
        return distance;
    }

    private double moveTime(double distance, double rpm, double circumferance) {
        return (distance / (rpm * circumferance) * 1000);
    }

    private double turnTime(double degrees) {
        return ((degrees / TURN_PER_SECOND) *1000);
    }



}