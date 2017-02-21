package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;


/**
 * Created by Kernel Panic on 11/26/16.
 */
@Autonomous(name="Kernel Panic Autonomous Red", group="Kernel Panic")
public class KernelPanicAutonomousRed extends LinearOpMode {
    KernelPanicPlatform robot = new KernelPanicPlatform();
    TwoArmButtonPusher  twoArmButtonPusher = new TwoArmButtonPusher();
    // Headings are the desired Headings for that step, expressed in "true" values
    public static int[] Headings = {
            // Original 309, 359, 315, 0, 0, 0
            311, 359, 317, 0, 0, 0
    };
    public static int[] rotationDir = {
            -20, 20, -20, 0, 0, 0
    };
    // This is a debugging and testing vector.
    public int[] desiredHeading = {
            0,    0,   0,   0,  0
    };
    // This is a debugging and testing vector.
    public int[] actualHeading = {
            0,    0,   0,   0,  0
    };

    public int red[] = {0, 0, 0, 0, 0, 0};
    public int blue[] = {0, 0, 0, 0, 0, 0};
    public int green[] = {0, 0, 0, 0, 0, 0};

    public static int prevHeading = 0;
    public static int newHeading = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        int currentHeading;
        int nextHeading;
        int leg = 0;

        boolean continueForward = true;


        robot.init(hardwareMap);
        twoArmButtonPusher.setParams(robot.frontServo, robot.backServo, robot.colorSide,
                robot.frontTouch, robot.backTouch, 2700,
                robot.SERVO_EXTEND_POWER, robot.SERVO_RETRACT_POWER,
                robot.SERVO_STOP_POWER, this);


        //Clean this up because of drive class
        DcMotor[] leftMotors = new DcMotor[]{robot.leftMotorFront, robot.leftMotorBack};
        DcMotor[] rightMotors = new DcMotor[]{robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        myDrive.setParams(12.5, 2, 79.5, .25, .25, .25, 1, -1, robot.gyro, this);
        waitForStart();

        opModeIsActive();

        // Move forward some
        myDrive.moveForward(12, -0.3);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();

        }
        myDrive.allStop();

        //Try to turn to 307 degrees
        myDrive.gyroTurn3(Headings[0], rotationDir[0]);
        myDrive.allStop();

        telemetry.addData("expected heading", Headings[0]);
        telemetry.addData("actual heading", robot.gyro.getHeading());
        telemetry.update();

        //Move forward towards the beacons, hit the wall, then back off
        myDrive.moveForward(48, -0.5);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();
        myDrive.moveForward(3, 0.5);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();
        safeSleep(2000);


        // Turn back to original heading..... May want to modify for turn to 350, move forward tiny amount, turn to 0
        myDrive.gyroTurn3(Headings[1], rotationDir[1]);
        myDrive.allStop();
        telemetry.addData("Expected Heading1  ", "%d", Headings[1]);
        telemetry.addData("Actual Heading1 ", robot.gyro.getHeading());
        telemetry.update();

        long mytime = System.currentTimeMillis();
        long loopingtime = 0;
        myDrive.driveMove(-.2, 0);
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

        myDrive.moveForward(8, 0.20);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();

        //telemetry.addData("Expected Heading1  ", "%d", Headings[1]);
        //telemetry.addData("Actual Heading1 ", robot.gyro.getHeading());

        //telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
        //telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
        //telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
        //telemetry.update();
        safeSleep(500);
        //Push Red button
        twoArmButtonPusher.pushRed();

        myDrive.moveForward(20.1, -0.8);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();

        //Temp sleep for test
        //safeSleep(1000);  //Get past first white line

        mytime = System.currentTimeMillis();
        loopingtime = 0;
        myDrive.driveMove(-.2, 0);
        //safeSleep(1000);  //Get past first white line
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

        myDrive.moveForward(8, 0.20);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();

        //desiredHeading[3] = Headings[1];
        //actualHeading[3] = robot.gyro.getHeading();
        //Red Autonomous
        //dataDump();
        //SystemClock.sleep(1000); //Give time to look at data

        //telemetry.addData("Side 1     Red   ", "%d", robot.colorSide.red());
        //telemetry.addData("Side 1     Green ", "%d", robot.colorSide.green());
        //telemetry.addData("Side 1     Blue  ", "%d", robot.colorSide.blue());
        safeSleep(500);
        twoArmButtonPusher.pushRed();

        myDrive.gyroTurn3(Headings[2], rotationDir[2]);
        myDrive.allStop();



        myDrive.moveForward(40, 0.9);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();

        safeSleep(1000);

        myDrive.moveForward(4, 0.3);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();




        telemetry.update();

        while(opModeIsActive()) {
            idle();
        }   // New opModeActive()

        myDrive.allStop();
        robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        robot.backServo.setPower(robot.SERVO_STOP_POWER);



    }

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
        telemetry.addData("Bottom      Red   ", "%d", robot.colorBottom.red());
        telemetry.addData("Bottom      Green ", "%d", robot.colorBottom.green());
        telemetry.addData("Bottom      Blue  ", "%d", robot.colorBottom.blue());
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
