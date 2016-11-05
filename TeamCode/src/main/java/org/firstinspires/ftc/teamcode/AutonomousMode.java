package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * Created by judenki on 11/5/16.
 */

@Autonomous(name="Juden-Ki Autonomous 1", group="Juden-Ki")
public class AutonomousMode extends LinearOpMode {
    //RobotTypeX robot;
    HardwareK9bot robot = new HardwareK9bot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        // Start off going
        robot.leftMotor.setPower(0.5);
        robot.rightMotor.setPower(0.25);

        while (opModeIsActive()) {
            if( robot.color.red() > 2 && robot.color.green() > 2 && robot.color.blue() > 2 ) {
                robot.leftMotor.setPower(0.0);
                robot.rightMotor.setPower(0.0);
            }
            if( robot.color.red() > 1 && robot.color.green() < 1  && robot.color.blue() < 1 ) {
                robot.leftMotor.setPower(0.5);
                robot.rightMotor.setPower(0.25);
            }
        }
    }
}
