package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * Created by judenki on 11/12/16.
 */

//@Autonomous(name="Autonomous Drive Test", group="Juden-Ki")
public class AutoDriveTest extends LinearOpMode {
    //RobotTypeX robot;
    HardwareK9bot robot = new HardwareK9bot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotor };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotor };
        Drive myDrive = new Drive(leftMotors, rightMotors);

        // Move 12 inches
        myDrive.moveForward(12, 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        //Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }
        // Move 72 inches stop when we see white line
        myDrive.moveForward(72, 0.8);
        while ((myDrive.motorsRunning() == true) && (robot.color.red() < 2 && robot.color.green() < 2 && robot.color.blue() < 2 )) {
            myDrive.update();
        }
        myDrive.allStop();

        //Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }




        while(opModeIsActive()) {
            idle();
        }
    }
}
