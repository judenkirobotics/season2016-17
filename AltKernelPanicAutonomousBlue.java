package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Howard 20170225
 */
@Autonomous(name="Alt Kernel Panic Autonomous Blue", group="Kernel Panic")
public class AltKernelPanicAutonomousBlue extends LinearOpMode {
    final double frameRate = 20;
    final double sensorFrameRate = 7;
    final double telemetryFrameRate = 1000;
    private CRServo frontServo     = null;
    private CRServo backServo      = null;
    private ColorSensor colorSide  = null;
    private TouchSensor frontTouch = null;
    private TouchSensor backTouch  = null;
    private GyroSensor driveGyro   = null;


    KernelPanicPlatform robot = new KernelPanicPlatform();
    TwoArmButtonPusher twoArmButtonPusher = new TwoArmButtonPusher();
    // broadly scoped sensor definitions.  Should be a populated data structure!
    int foundRed = 0;
    int foundBlue = 0;
    int foundGreen = 0;
    int foundWhite = 0;
    int foundSideRed = 0;
    int foundSideBlue = 0;
    int foundSideGreen = 0;
    int foundSideWhite = 0;
    int frontPressed;
    int backPressed;
    int currHeading = 0;
    /***
     * our legs go like this
     * 0: head out from the wall
     * 1: 45 deg turn to right
     * 2: straight toward other wall
     * 3: back up a bit
     * 4: turn to 0
     * 5: forward to white line
     * 6: adjust position
     * 7: punch button
     * 8: forward to white line 2
     * 9: adjust position
     * 10: punch buttons 2
     * 11: turn to 45 deg
     * 12: retreat at flank speed to cap ball
     * 13: final position adjust
     */
    // --------------- turn leg information here ---------------------
    int[] Headings = {
            0, 43,  0,  0,  0,  0,  0,  0,  0,  0,  0,  42,  0,  0
         // 0   1   2   3   4   5   6   7   8   9   10  11  12  13
    };
    int[] rotationPwr = {
            0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 20, 0, 0
         // 0   1  2  3   4   5  6  7  8  9 10  11  12 13
    };
    //  ---------------------  straightline stuff below ----------------
    double[] straightDist = {
            0, 20,  0,  0,-30,  0,  0,  0,  0,  0,   0, 20,  0,  0
         // 0   1   2   3   4   5   6   7   8   9   10  11  12  13
    };
    double[] straightPwr = {
            0, 20,  0,  0,-30,  0,  0,  0,  0,  0,   0, 20,  0,  0
            // 0   1   2   3   4   5   6   7   8   9   10  11  12  13
    };
    int leg = 0;
    double prevDist = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        DcMotor[] leftMotors = new DcMotor[]{robot.leftMotorFront, robot.leftMotorBack};
        DcMotor[] rightMotors = new DcMotor[]{robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);


        twoArmButtonPusher.setParams(robot.frontServo, robot.backServo, robot.colorSide,
                robot.frontTouch, robot.backTouch, 2700,
                robot.SERVO_EXTEND_POWER, robot.SERVO_RETRACT_POWER,
                robot.SERVO_STOP_POWER, this);
        myDrive.setParams(12.5, 2, 79.5, .25, .25, .25, 1, -1, robot.gyro, this);
        waitForStart();
        /***************************************************************************
         *            Everything below here happens after we press START           *
         ***************************************************************************/
        opModeIsActive();
        double lastFrame1 = System.currentTimeMillis();
        double lastFrame2 = System.currentTimeMillis() + frameRate/2;
        int overRun1 = 0;
        int overRun2 = 0;
        int skipped1 = 0;
        int skipped2 = 0;
        double currTime = System.currentTimeMillis();
        double legTime = currTime;
        double lastTelemetry = currTime;
        while (opModeIsActive()) {
            if (currTime - lastFrame1 > frameRate) {
                // autotasks processes each leg
                if (currTime - lastFrame1 > (2 * frameRate)) skipped1++;
                if (autoTasks(legTime, currTime) > 0) {
                    leg = (leg < 65535) ? leg++ : leg;
                    lastFrame1 = currTime;
                    myDrive.allStop();
                }
                else
                {
                    //myDrive.  set motor power left, set motor power right.
                }
            }
            if (currTime - lastFrame2 > sensorFrameRate){
                // this will read the sensors
                if (currTime - lastFrame2 > (2*sensorFrameRate)) skipped1++;
                if ( readSensors(currTime) > 0) lastFrame2 = currTime;
            }
            if (currTime - lastTelemetry > telemetryFrameRate) {
                //Display diagnostic data
                telemetry.update();
                lastTelemetry = currTime;
            }

            // Set motor power
            //while ((myDrive.motorsRunning()) && opModeIsActive()) myDrive.update();
            //myDrive.allStop();


        }
        myDrive.allStop();
        robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        robot.backServo.setPower(robot.SERVO_STOP_POWER);
    }
    public int autoTasks(double legStart, double currTime){
        int taskComplete = 0;
        /***
         * our legs go like this
         * 0: head out from the wall
         * 1: 45 deg turn to right
         * 2: straight toward other wall
         * 3: back up a bit
         * 4: turn to 0
         * 5: forward to white line
         * 6: adjust position
         * 7: punch button
         * 8: forward to white line 2
         * 9: adjust position
         * 10: punch buttons 2
         * 11: turn to 45 deg
         * 12: retreat at flank speed to cap ball
         * 13: final position adjust
         */
        switch (leg) {
            case 0:case 2:case 3:case 5:case 12:case 13: {
                // straight runs to a direction
                straightDist[leg] = driveStraight(straightDist[leg],straightPwr[leg],legStart,currTime);
                taskComplete = (int)((double)1-straightPwr[leg]);
            }
            case 1:case 4:case 11: {
                // these are the turning cases
                rotationPwr[leg] = gyroTurn5(Headings[leg], rotationPwr[leg], currHeading, legStart);
                // gyroturn 5 needs to modulate power and set power to zero when it times out
                // or has achieved the desired heading.
                taskComplete = 1-rotationPwr[leg];

            }
            case 6:case 9: {
                // find white line
                // need to add findWhiteLine. Will pass bottom color sensor, pwr setting, clock
                taskComplete = findWhiteLine();
            }
            case 7:case 10: {
                // punch buttons
                // also need to add the punchButtons. will pass clock, side color.
                taskComplete = pushBlue();
            }
            default: {
                // calling routine should set ALL STOP if task is complete.
                // allows starting next task from a known start point.
                taskComplete = 1;
            }
        }
        return(taskComplete);
    }
    public int readSensors(double startTime) {
        int tasksComplete = 0;
        int Q = 10;
        foundRed = (robot.colorBottom.red() > 10)? 1 : 0;
        foundBlue = Math.max(0,(robot.colorBottom.blue()+foundBlue - Q));
        foundGreen = Math.max(0,(robot.colorBottom.green()+foundGreen - Q));
        foundWhite = Math.max(0,(foundRed + foundBlue + foundGreen + foundWhite - Q));
        foundSideRed = Math.max(0,(robot.colorSide.red() - Q));
        foundSideBlue = Math.max(0,(robot.colorSide.blue() - Q));
        foundSideGreen = Math.max(0,(robot.colorSide.green() - Q));
        foundSideWhite = Math.max(0,(foundSideRed+foundSideBlue+foundSideGreen+foundSideWhite -Q));
        frontPressed = (frontTouch.isPressed())? Math.min(Q,frontPressed++) : Math.max(0,frontPressed--);
        backPressed = (backTouch.isPressed())?Math.min(Q,backPressed++) : Math.max(0,backPressed--);;
        currHeading = driveGyro.getHeading();
        tasksComplete = ((System.currentTimeMillis() - startTime) > frameRate) ? 0 : 1;
        return (tasksComplete);
    }
    public double driveStraight(double pwr, double duration, double time, double startTime)
    {
        // going to try a power & time based means of setting power.  Very simple.
        // the distance is an abstract number, but ultimately relates to time
        // we pass in the leg start time and the current time.
        for (DcMotor dcm : leftMotors){
            dcm.setPower( - pwr);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( pwr);
        }


        //long moveTime(double distance, double rpm, double circumferance) {
        //   return (long)(distance / (rpm * circumferance) * 1000);
        return (1-(int)duration - time - startTime);
    }
    public int gyroTurn5 (int target, int clockwise, int current, double startTime){

        return 0;
    }
    public int pushBlue(){
        return 0;
    }
    public int findWhiteLine() {
        return 0;
    }

/*
            // Move forward some
            myDrive.moveForward(12, 0.3);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();

            //Try to turn 45 degrees
            myDrive.gyroTurn3B(Headings[0], rotationDir[0]);
            myDrive.allStop();


            myDrive.moveForward(48, 0.5);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();
            safeSleep(200);

            myDrive.moveForward(3, -0.5);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }

            // THIS ALLOWS THE GYRO TO DEBOUNCE - LITERALLY - AFTER WALL CONTACT
            myDrive.allStop();
            safeSleep(2800);

            //Turn back to original heading
            myDrive.gyroTurn3B(Headings[1], rotationDir[1]);
            myDrive.allStop();
            telemetry.addData("Expected Heading1  ", "%d", Headings[1]);
            telemetry.addData("Actual Heading1 ", robot.gyro.getHeading());
            telemetry.update();


            long mytime = System.currentTimeMillis();
            long loopingtime = 0;
            myDrive.driveMove(.2, 0);
            boolean keepmoving = true;
            while ((keepmoving == true) && opModeIsActive()) {
                if (((robot.colorBottom.red() > 10) &&
                        (robot.colorBottom.blue() > 10) &&
                        (robot.colorBottom.green() > 10)) ||
                        (loopingtime > 5000)) {
                    myDrive.allStop();
                    keepmoving = false;
                }
                loopingtime = System.currentTimeMillis() - mytime;
            }
            myDrive.allStop();

            myDrive.moveForward(8, -0.20);  //8
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();

            //Wait a bit to make sure color saturates
            safeSleep(500);
            //Push blue buttons
            twoArmButtonPusher.pushBlue();

            myDrive.moveForward(20.1, 0.8);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();


            mytime = System.currentTimeMillis();
            loopingtime = 0;
            myDrive.driveMove(.2, 0);
            keepmoving = true;
            while ((keepmoving == true) && opModeIsActive()) {
                if (((robot.colorBottom.red() > 10) &&
                        (robot.colorBottom.blue() > 10) &&
                        (robot.colorBottom.green() > 10)) ||
                        (loopingtime > 5000)) {
                    myDrive.allStop();
                    keepmoving = false;
                }
                loopingtime = System.currentTimeMillis() - mytime;
            }
            myDrive.allStop();

            myDrive.moveForward(8, -0.2);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();

            telemetry.addData("Expected Heading1  ", "%d", Headings[1]);
            telemetry.addData("Actual Heading1 ", robot.gyro.getHeading());

            telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
            telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
            telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
            safeSleep(500);
            //Push blue buttons
            twoArmButtonPusher.pushBlue();

            //Turn towards center and drive to knock off ball
            myDrive.gyroTurn3B(Headings[2], rotationDir[2]);

            myDrive.allStop();

            //Drive towards cap ball as fast as possible
            myDrive.moveForward(43, -0.9);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();

            //If time allows briefly pause and then bump the ball
            safeSleep(1000);
            myDrive.moveForward(4, -0.3);*/
    public int newHeading (int currentHeading, int turnHeading) {
        int tempHeading;

        tempHeading = currentHeading + turnHeading;

        if (tempHeading > 360)
            tempHeading = tempHeading -360;

        if (tempHeading < 0)
            tempHeading = tempHeading + 360;

        return (tempHeading);


    }

    public void dataDump() {
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
        telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
        telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
    }

    public void safeSleep (long duration) {
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - duration < currentTime) && opModeIsActive()) {
            //kill some time
            // idle();
        }
    }


}
