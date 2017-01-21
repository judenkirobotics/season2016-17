package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by judenki on 1/3/17.
 *
 * Incorporate the shoot the ball routine from all of teh autonomous modes and driver mode.
 */

public class ShootTheBall {
    private int touchSensorDelay=500;
    private int motorAbortDelay=2000;
    private int motorStopDelay=150;

    public void shoot (DcMotor mot, TouchSensor touch, LinearOpMode myMode)  {

        //Set throwing motor to full power
        mot.setPower(.9);

        //Wait some time for it to cycle past the touch sensor
        double currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentTime < touchSensorDelay) && myMode.opModeIsActive()) {
            //Kill some time

        }

        //Reduce motor power
        mot.setPower(.50);
        currentTime = System.currentTimeMillis();
        while ((touch.isPressed() != true) && myMode.opModeIsActive() && (  System.currentTimeMillis() - currentTime < motorAbortDelay)) {
            //kill some time
        }
        currentTime = System.currentTimeMillis();
        while((System.currentTimeMillis() - currentTime < motorStopDelay) && myMode.opModeIsActive())
        mot.setPower(0);
    }

    public void prime(DcMotor mot, TouchSensor touch, LinearOpMode myMode) {
        double startPrime = System.currentTimeMillis();
        while((touch.isPressed() != true)     &&
              (myMode.opModeIsActive())       &&
              (System.currentTimeMillis() - startPrime < 5000)) {
            // drive the prime motor
            mot.setPower(.50);
        }
        // TRUE, so stop motor
        mot.setPower(0);
    }

    public void setTouchDelay(int delayTime) {
        touchSensorDelay=delayTime;
    }
    public void setAbortDelay(int delayTime) {
        motorAbortDelay=delayTime;
    }
    public void setMotorStopDelay(int delayTime) {
        motorStopDelay=delayTime;
    }
}

