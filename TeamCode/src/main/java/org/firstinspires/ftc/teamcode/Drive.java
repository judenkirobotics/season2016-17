package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

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
    public static int prevHeading = 176;
    public static int newHeading = 178;
    private DcMotor[] leftMotors   = null;
    private DcMotor[] rightMotors  = null;

    private double MIN_DRIVE_DISTANCE = 0.0;
    private double MAX_DRIVE_DISTANCE = 120.0;

    private double FAST_POWER          = 0.6;
    private double SLOW_POWER          = 0.3;
    private double CORRECTION_POWER    = 0.15;
    private double RIGHT_SIGN          = 1;
    private double LEFT_SIGN           =-1;
    public static boolean RIGHT_TURN   = true;
    public static boolean LEFT_TURN    = false;
    private GyroSensor driveGyro       = null;
    private LinearOpMode myMode        = null;





    private double WHEEL_CIRC    = 13;
    private double WHEEL_RPM     = 2;   //Misnamed, fix should be RPS second not minute

    private double TURN_PER_SECOND = 79.5;

    private long driveStopTime  = 0;
    private long turnStartTime = 0;
    private long turnTotalTime = 0;
    final private double powerTable[] = {
          0.0,  0.2, 0.3, 0.35, 0.4, 0.45, 0.48, 0.5, 0.55, 0.6
    };
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
            //dcm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //dcm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        for (DcMotor dcm : leftMotors){
            dcm.setPower( - power);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( power);
        }

        driveStopTime = time + SystemClock.elapsedRealtime();
        motorsStopped = false;


    }


    public void gyroTurn4(int newHeading, int clockwise) {
        int currentHeading;
        int deltaHeading;

        // Call the base gyro turn code
        gyroTurn3(newHeading, clockwise);

        //Kill a little time then check to see if we are at the correct heading
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - 250 < currentTime)  && myMode.opModeIsActive()) {
            //kill some time
            // idle();
        }

        //check if at correct heading
        currentHeading = driveGyro.getHeading();
        deltaHeading   = newHeading-currentHeading;
        if (deltaHeading < -2) {
            deltaHeading = deltaHeading + 360;
        }


        // THis code works, problem is gyroTurn3() will always rotate at least 6 degrees.
        if (deltaHeading < -2) {
            //Over rotated, try a short goose of the robot
            myMode.telemetry.addData("Over rotate", deltaHeading);
            driveMove(0,.7*clockwise * -1);
            //gyroTurn3(newHeading, clockwise);
        }
        else if (deltaHeading > 2) {
            //Under rotate, try a short goose of the robot
            myMode.telemetry.addData("Over rotate", deltaHeading);
            driveMove(0, .7 * clockwise);
            //gyroTurn3(newHeading,clockwise*-1);
        }
        else {
            //This bed feels just right
        }

        currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - 250 < currentTime)  && myMode.opModeIsActive()) {
            //kill some time
            // idle();
        }


        /*deltaHeading = getNeededTurn(currentHeading, newHeading, clockwise);

        // Check to see if we over or under rotated by more than two degrees, if so correct.
        if(deltaHeading > 180) {
            myMode.telemetry.addData("Entering retry turn with heading", deltaHeading);
            if (deltaHeading < 358) {
                clockwise = clockwise * -1;
                myMode.telemetry.addData("Overrotate", newHeading);
                myMode.telemetry.update();
                SystemClock.sleep(10000);
                gyroTurn3(newHeading, clockwise);
            }
        }
        else if(deltaHeading > 2){
            myMode.telemetry.addData("Underrotate", newHeading);
            myMode.telemetry.update();
            SystemClock.sleep(10000);
            gyroTurn3(newHeading, clockwise);
        }
        else {
            //Within two degrees do nothing.
        }*/

    }

    /*
     *  Make a method to consolidate all of the different gyro based turns the students have come up with.
     *  Accept the new absolute heading and direction of turn.   Initialization routine must be called
     *  first to set parameters fastPower, slowPower, correctionPower, rightSign, leftSign.
     */
    public void gyroTurn3B(int newHeading, int clockwise){
        int currHeading = driveGyro.getHeading();
        int accumTurn = 0;
        long turnSegmentTime = 0;
        long segStartTime = 0;
        double turnRate = 0;
        double powerAdjustAmount = 0.05;

        int segDegrees = 0;
        int cw = (clockwise < 0) ? -1 : 1;
        int transit = (((currHeading > newHeading) && (cw > 0)) ||
                ((currHeading < newHeading) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newHeading) + ((-1*cw)*currHeading));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        prevHeading = currHeading;
        turnStartTime = System.currentTimeMillis();
        turnTotalTime = 0;
        segStartTime = turnStartTime;
        int startSegDegrees = accumTurn;
        double ratePowerAdjust = 0;
        double powerLevel = 0.20; // as good a place as any to start
        int upCount=0;
        int downCount=0;


        while((accumTurn < desiredRotation) &&
                (clockwise != 0)              &&
                (turnTotalTime < 30000)       &&
                (myMode.opModeIsActive()))      {

            long currTime = System.currentTimeMillis();
            turnTotalTime = currTime - turnStartTime;
            turnSegmentTime = currTime - segStartTime;
            segDegrees = accumTurn - startSegDegrees;


            if (turnSegmentTime > 250) {
                startSegDegrees = accumTurn;
                segStartTime = currTime;
                turnRate = 1000.0 * (double)segDegrees / (double)turnSegmentTime;
                if (((turnRate > 10.0) && (desiredRotation - accumTurn) <= 3)  ||
                        (turnRate > Math.abs(clockwise+5))){
                    ratePowerAdjust = ratePowerAdjust - 0.05;
                    upCount++;
                }
                else if (turnRate < Math.abs(clockwise)) {
                    //ratePowerAdjust = ratePowerAdjust + 0.05;
                    ratePowerAdjust = ratePowerAdjust + powerAdjustAmount;
                    powerAdjustAmount = (powerAdjustAmount > 0.01) ? powerAdjustAmount - 0.01 : 0;
                    downCount++;
                }

            }
            double turnDir = (clockwise > 0) ? RIGHT_SIGN : LEFT_SIGN;
            powerLevel = (powerLevel + ratePowerAdjust);
            powerLevel = (powerLevel > 0.5)? 0.5 : powerLevel;  //Clamp upper power original .7
            powerLevel = (powerLevel < 0.26)? 0.26 : powerLevel;  //Clamp lower power original .2
            driveMove(0, turnDir * powerLevel);

            currHeading = driveGyro.getHeading();
            if (Math.abs(prevHeading - currHeading) > 350) {
                if (prevHeading < currHeading) {
                    accumTurn += prevHeading + 360 - currHeading;
                }
                else {
                    accumTurn += currHeading + 360 - prevHeading;
                }
            }
            else {
                accumTurn += Math.abs(prevHeading - currHeading);
            }
            prevHeading = currHeading;

        }

        //Always command a stop before exiting
        driveMove(0,0);


        // Lets look at some of the results
        myMode.telemetry.addData("Power LEvel ", "%f", powerLevel);
        myMode.telemetry.addData("UpCount ", "%d", upCount);
        myMode.telemetry.addData("DownCount ", "%d", downCount);
    }


    public void gyroTurn3(int newHeading, int clockwise){
        int currHeading = driveGyro.getHeading();
        int accumTurn = 0;
        long turnSegmentTime = 0;
        long segStartTime = 0;
        double turnRate = 0;
        double powerAdjustAmount = 0.05;

        int segDegrees = 0;
        int cw = (clockwise < 0) ? -1 : 1;
        int transit = (((currHeading > newHeading) && (cw > 0)) ||
                       ((currHeading < newHeading) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newHeading) + ((-1*cw)*currHeading));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        prevHeading = currHeading;
        turnStartTime = System.currentTimeMillis();
        turnTotalTime = 0;
        segStartTime = turnStartTime;
        int startSegDegrees = accumTurn;
        double ratePowerAdjust = 0;
        double powerLevel = 0.20; // as good a place as any to start
        int upCount=0;
        int downCount=0;


        while((accumTurn < desiredRotation) &&
              (clockwise != 0)              &&
              (turnTotalTime < 30000)       &&
              (myMode.opModeIsActive()))      {

            long currTime = System.currentTimeMillis();
            turnTotalTime = currTime - turnStartTime;
            turnSegmentTime = currTime - segStartTime;
            segDegrees = accumTurn - startSegDegrees;


            if (turnSegmentTime > 250) {
                startSegDegrees = accumTurn;
                segStartTime = currTime;
                turnRate = 1000.0 * (double)segDegrees / (double)turnSegmentTime;
                if (((turnRate > 10.0) && (desiredRotation - accumTurn) <= 15)  ||
                     (turnRate > Math.abs(clockwise+5))){
                    ratePowerAdjust = ratePowerAdjust - 0.1;
                    upCount++;
                }
                else if (turnRate < Math.abs(clockwise)) {
                    //ratePowerAdjust = ratePowerAdjust + 0.05;
                    ratePowerAdjust = ratePowerAdjust + powerAdjustAmount;
                    powerAdjustAmount = (powerAdjustAmount > 0.01) ? powerAdjustAmount - 0.01 : 0;
                    downCount++;
                }

            }
            double turnDir = (clockwise > 0) ? RIGHT_SIGN : LEFT_SIGN;
            powerLevel = (powerLevel + ratePowerAdjust);
            powerLevel = (powerLevel > 0.5)? 0.5 : powerLevel;  //Clamp upper power original .7
            powerLevel = (powerLevel < 0.20)? 0.20 : powerLevel;  //Clamp lower power original .2
            driveMove(0, turnDir * powerLevel);

            currHeading = driveGyro.getHeading();
            if (Math.abs(prevHeading - currHeading) > 350) {
                if (prevHeading < currHeading) {
                    accumTurn += prevHeading + 360 - currHeading;
                }
                else {
                    accumTurn += currHeading + 360 - prevHeading;
                }
            }
            else {
                accumTurn += Math.abs(prevHeading - currHeading);
            }
            prevHeading = currHeading;

        }

        //Always command a stop before exiting
        driveMove(0,0);


        // Lets look at some of the results
        myMode.telemetry.addData("Power LEvel ", "%f", powerLevel);
        myMode.telemetry.addData("UpCount ", "%d", upCount);
        myMode.telemetry.addData("DownCount ", "%d", downCount);
    }


    public void gyroTurn2(int newHeading, int clockwise){
        int currHeading = driveGyro.getHeading();
        int accumTurn = 0;
        int cw = (clockwise < 0) ? -1 : 1;
        int transit = (((currHeading > newHeading) && (cw > 0)) ||
                ((currHeading < newHeading) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newHeading) + ((-1*cw)*currHeading));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        prevHeading = currHeading;

        while((accumTurn < desiredRotation) &&
                (clockwise != 0) &&
                (myMode.opModeIsActive())){
            double turnDir = (clockwise > 0) ? RIGHT_SIGN : LEFT_SIGN;
            double powerLevel = ((desiredRotation - accumTurn) > 15) ? FAST_POWER : SLOW_POWER;
            powerLevel = ((desiredRotation - accumTurn) > 3) ? powerLevel : CORRECTION_POWER;
            driveMove(0, turnDir * powerLevel);
            //numberSteps[leg]++;
            currHeading = driveGyro.getHeading();
            if (Math.abs(prevHeading - currHeading) > 350) {
                if (prevHeading < currHeading) {
                    accumTurn += prevHeading + 360 - currHeading;
                }
                else {
                    accumTurn += currHeading + 360 - prevHeading;
                }
            }
            else {
                accumTurn += Math.abs(prevHeading - currHeading);
            }
            prevHeading = currHeading;
        }
        driveMove(0,0);
    }
    public void gyroTurn(int newHeading, boolean direction) {

        /*
         * Four cases to consider.  First determine if new Heading is greater or less than current heading.
         * Then determine if they turn right or left to reach new heading.
         */


        if (newHeading > driveGyro.getHeading()) {
            if (direction == RIGHT_TURN) {
                driveMove(0,RIGHT_SIGN*FAST_POWER);
                while ((newHeading - driveGyro.getHeading()>20) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,RIGHT_SIGN*SLOW_POWER);
                while ((newHeading - driveGyro.getHeading()>0) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,0);
                if (newHeading - driveGyro.getHeading() < 0) {
                    driveMove(0,LEFT_SIGN*CORRECTION_POWER);
                    while ((newHeading - driveGyro.getHeading() < 0) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }

                driveMove(0,0);

            }
            // Turn left to reach new heading must handle crossing zero
            else {
                if (driveGyro.getHeading() > 0) {
                    //Pass zero to get to a positive heading number
                    driveMove(0,LEFT_SIGN*FAST_POWER);
                    while((driveGyro.getHeading() > 10) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                    driveMove(0,LEFT_SIGN*SLOW_POWER);
                    while((driveGyro.getHeading() > 0) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }
                else {
                    driveMove(0,LEFT_SIGN*SLOW_POWER);
                    while((driveGyro.getHeading() < 10) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }

                //Now that past zero keep turning to new heading
                driveMove(0,LEFT_SIGN*FAST_POWER);
                while ((driveGyro.getHeading() > newHeading+10) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,LEFT_SIGN*SLOW_POWER);
                while ((driveGyro.getHeading() > newHeading) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                if (driveGyro.getHeading() - newHeading < 0) {
                    driveMove(0,RIGHT_SIGN*CORRECTION_POWER);
                    while ((driveGyro.getHeading() - newHeading < 0) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }
                driveMove(0,0);

            }

        }


        // New heading is less than current heading
        else {
            if (direction == RIGHT_TURN) {
                //Handle crossing zero
                driveMove(0,RIGHT_SIGN*FAST_POWER);
                while ((driveGyro.getHeading() < 350) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,RIGHT_SIGN*SLOW_POWER);
                while ((driveGyro.getHeading() > 10) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,0);

                // Turn to new heading
                driveMove(0,RIGHT_SIGN*FAST_POWER);
                while ((newHeading - driveGyro.getHeading() > 10) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,RIGHT_SIGN*SLOW_POWER);
                while ((newHeading - driveGyro.getHeading() > 0) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                if (newHeading - driveGyro.getHeading() < 0) {
                    driveMove(0,LEFT_SIGN*CORRECTION_POWER);
                    while ((newHeading - driveGyro.getHeading() < 0) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }
                driveMove(0,0);

            }
            // Turn left to reach new heading
            else {
                driveMove(0,LEFT_SIGN*FAST_POWER);
                while ((driveGyro.getHeading() - newHeading > 10) && myMode.opModeIsActive()) {
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,LEFT_SIGN*SLOW_POWER);
                while ((driveGyro.getHeading() - newHeading > 00)  && myMode.opModeIsActive()){
                    // Add time based check to keep loop from getting stuck
                }
                driveMove(0,0);
                if (driveGyro.getHeading() - newHeading < 0) {
                    driveMove(0,RIGHT_SIGN*CORRECTION_POWER);
                    while ((driveGyro.getHeading() - newHeading < 0) && myMode.opModeIsActive()) {
                        // Add time based check to keep loop from getting stuck
                    }
                }
                driveMove(0,0);
            }

        }
    }

    /***********************************************************
     **          calculate necessary turn amount              **
     **********************************************************/
    public static int getNeededTurn(int isNow, int want, int cw) {
        cw = (cw < 0) ? -1 : 1;
        int transit = (((isNow > want) && (cw > 0)) ||
                ((isNow < want) && (cw < 0))) ?  360 : 0;
        int neededTurn = Math.abs(transit + (cw*want) + ((-1*cw)*isNow));
        neededTurn = (neededTurn > 360) ? neededTurn - 360 : neededTurn;
        return (neededTurn);
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

    public void setParams(double wheelCirc, double wheelRPM, double turnPerSecond,
                          double fastPower, double slowPower, double correctionPower,
                          double rightSign, double leftSign, GyroSensor gyro,
                          LinearOpMode operationMode) {
        WHEEL_CIRC = wheelCirc;
        WHEEL_RPM = wheelRPM;
        TURN_PER_SECOND = turnPerSecond;
        FAST_POWER          = fastPower;
        SLOW_POWER          = slowPower;
        CORRECTION_POWER    = correctionPower;
        RIGHT_SIGN          = rightSign;
        LEFT_SIGN           = leftSign;
        driveGyro           = gyro;
        myMode = operationMode;


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
