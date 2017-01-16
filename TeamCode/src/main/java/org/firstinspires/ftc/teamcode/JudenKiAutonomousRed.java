
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by judenki on 11/26/16.
 */
@Autonomous(name="Juden-Ki Autonomous Red", group="Juden-Ki")
public class JudenKiAutonomousRed  extends LinearOpMode {
    //Juden Ki Launching Robot
    JudenKiPlatform robot = new JudenKiPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        int currentHeading;
        int nextHeading;

        robot.init(hardwareMap);
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        myDrive.setParams(12.5, 1.5, 79.5);  //need to change these later
        waitForStart();
        ShootTheBall particle = new ShootTheBall();

// Move forward some
        myDrive.moveForward(3 , -0.6 );
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }

        //start turn
        myDrive.driveMove(0, -0.5);

        currentHeading = robot.gyro.getHeading();
        while ((currentHeading > 327) || (currentHeading < 323) && opModeIsActive()) {
            //Kill some time
            currentHeading = robot.gyro.getHeading();
        }
        myDrive.allStop();

        //Shoot Balls (2)
        particle.shoot(robot.catapultMotor, robot.touchCat, this);
        particle.shoot(robot.catapultMotor, robot.touchCat, this);


        //start turn
        myDrive.driveMove(0, 0.5);

        currentHeading = robot.gyro.getHeading();
        while ((currentHeading > 342) || (currentHeading < 338) && opModeIsActive()) {
            //Kill some time
            currentHeading = robot.gyro.getHeading();
        }
        myDrive.allStop();

        // Move forward some
        myDrive.moveForward(25, -0.6 );
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }

        SystemClock.sleep(5000);

        // Move forward some
        myDrive.moveForward(10, -0.6 );
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }


        while(opModeIsActive()) {
            idle();
        }

        myDrive.allStop();

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

    void dataDump() {
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
    }
}



