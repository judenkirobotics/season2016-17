package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * Created by judenki on 11/12/16.
 */

@Autonomous(name="Autonomous Drive Test", group="Juden-Ki")
public class AutoDriveTest extends LinearOpMode {
    //RobotTypeX robot;
    HardwareK9bot robot = new HardwareK9bot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        Drive myDrive = new Drive(robot.leftMotor, robot.rightMotor);

        // Move 12 inches
        myDrive.moveForward(12);

        //Sleep 3 seconds
        SystemClock.sleep(3000);

        // Move 6 inches
        myDrive.moveForward(6);

        //Sleep 3 seconds
        SystemClock.sleep(3000);

        // Move 3 inches
        myDrive.moveForward(3);

        //Sleep 3 seconds
        SystemClock.sleep(3000);

        //Turn 90 degrees
        myDrive.turn(90);

        //Sleep 3 seconds
        SystemClock.sleep(3000);

        //Turn -180 degrees
        myDrive.turn(-180);

        while(opModeIsActive()) {
            idle();
        }
    }
}
