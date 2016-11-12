package org.firstinspires.ftc.teamcode;

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
        myDrive.moveForward();
        while(opModeIsActive()) {
            idle();
        }
    }
}
