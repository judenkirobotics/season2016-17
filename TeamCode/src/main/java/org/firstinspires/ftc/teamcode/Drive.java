package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by judenki on 11/12/16.
 */

public class Drive {

    private DcMotor leftMotor   = null;
    private DcMotor rightMotor  = null;

    // Left and right are with respect to the robot
    public Drive( DcMotor _leftMotor, DcMotor _rightMotor ) {
        assert leftMotor != null;
        assert rightMotor != null;
        this.leftMotor = _leftMotor;
        this.rightMotor = _rightMotor;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn() {

    }
    public void moveForward() {
        // Move forward
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setPower(0.1);
        //leftMotor.setMaxSpeed(100);
        rightMotor.setPower(0.1);
        //rightMotor.setMaxSpeed(100);
        SystemClock.sleep(2000);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
    public void moveBackward() {

    }
}