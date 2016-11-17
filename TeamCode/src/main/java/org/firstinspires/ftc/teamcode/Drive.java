package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by judenki on 11/12/16.
 */

public class Drive {

    private DcMotor leftMotor   = null;
    private DcMotor rightMotor  = null;

    private double MIN_DRIVE_DISTANCE = 0.0;
    private double MAX_DRIVE_DISTANCE = 120.0;

    private double FORWARD_POWER =  0.8;
    private double REVERSE_POWER = -0.8;
    private double TURN_POWER    =  0.2;

    private double WHEEL_CIRC    = 2;
    private double WHEEL_RPM     = 3;

    private double TURN_PER_SECOND = 45;


    // Left and right are with respect to the robot
    public Drive( DcMotor _leftMotor, DcMotor _rightMotor ) {
        assert leftMotor != null;
        assert rightMotor != null;
        this.leftMotor = _leftMotor;
        this.rightMotor = _rightMotor;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  //Want to run with encoder but having issue
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turn(double degrees) {
        double time;
        double leftSign;
        double rightSign;

        if (degrees > 0) {
            rightSign = -1;
            leftSign  =  1;
            time = turnTime(degrees);
        }
        else {
            rightSign = 1;
            leftSign  = -1;
            time = turnTime(degrees) * -1;
        }



        leftMotor.setPower(leftSign * TURN_POWER);
        rightMotor.setPower(rightSign * TURN_POWER);

        SystemClock.sleep((long)(double)time);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);


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
        leftMotor.setPower(FORWARD_POWER);
        //leftMotor.setMaxSpeed(100);
        rightMotor.setPower(FORWARD_POWER);
        //rightMotor.setMaxSpeed(100);

        //SystemClock.sleep((long)(double)time);
        SystemClock.sleep((long)(double)time);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);


    }
    public void moveBackward(double distance) {
        double moveDistance;
        double   time;
        // Input distance will be in inches, perform a range check
        moveDistance = rangeCheck(distance);

        //Calculate how long to move.
        time =  moveTime(moveDistance, WHEEL_RPM, WHEEL_CIRC);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setPower(REVERSE_POWER);
        //leftMotor.setMaxSpeed(100);
        rightMotor.setPower(REVERSE_POWER);
        //rightMotor.setMaxSpeed(100);

        //SystemClock.sleep((long)(double)time);
        SystemClock.sleep((long)(double)time);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

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