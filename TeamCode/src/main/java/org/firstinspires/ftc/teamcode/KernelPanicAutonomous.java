package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Kernel Panic on 11/26/16.
 */
@Autonomous(name="Kernel Panic Autonomous", group="Kernel Panic")
public class KernelPanicAutonomous extends LinearOpMode {
    //Juden Ki Launching Robot
    KernelPanicPlatform robot = new KernelPanicPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        int currentHeading;
        int nextHeading;

        robot.init(hardwareMap);
        //Clean this up because of drive class
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        waitForStart();

        // Move forward some
        myDrive.moveForward(10, 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        //Try to turn 45 degrees
        /*currentHeading = robot.gyro.getHeading();
        nextHeading = newHeading(currentHeading, 45);
        myDrive.driveMove(0,0.25);
        while(robot.gyro.getHeading() <= nextHeading) {
            //Kill some time
            telemetry.addData("heading", robot.gyro.getHeading());
            telemetry.update();
        }
        myDrive.allStop();
*/
        //Move forward some more
        //myDrive.moveForward(24 , 0.8);
        //while (myDrive.motorsRunning() == true) {
        //    myDrive.update();
        //}

        while(opModeIsActive()) {
            idle();
        }

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


}
