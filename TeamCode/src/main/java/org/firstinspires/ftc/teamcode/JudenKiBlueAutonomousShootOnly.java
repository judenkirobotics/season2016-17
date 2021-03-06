package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by judenki on 11/26/16.
 */
@Autonomous(name="Juden-Ki Shoot Only Autonomous Blue", group="Juden-Ki")
public class JudenKiBlueAutonomousShootOnly   extends LinearOpMode {
    //Juden Ki Launching Robot
    JudenKiPlatform robot = new JudenKiPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        int currentHeading;
        int nextHeading;

        robot.init(hardwareMap);
        DcMotor[] leftMotors = new DcMotor[]{robot.leftMotorFront, robot.leftMotorBack};
        DcMotor[] rightMotors = new DcMotor[]{robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        myDrive.setParams(12.5, 1.5, 79.5);  //need to change these later
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say",  "Beyblades, Beyblades Let It Rip!");
        telemetry.update();
        waitForStart();
        ShootTheBall particle = new ShootTheBall();

        // Initialize catapult parameters
        particle.setAbortDelay(2000);
        particle.setTouchDelay(500);
        particle.setMotorStopDelay(151);

        //Move forward a small amount
        myDrive.moveForward(4, -0.6);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }


        //telemetry.addData("heading", robot.gyro.getHeading());
        //telemetry.update();
        //SystemClock.sleep(2000);
        //Turn to roughly 35 degrees
        myDrive.driveMove(0, 0.3);
        currentHeading = robot.gyro.getHeading();
        while ((currentHeading < 25) || (currentHeading > 31) && opModeIsActive()) {
            currentHeading = robot.gyro.getHeading();
        }
        myDrive.allStop();

        //Move forward a small amount
        myDrive.moveForward(4, -0.6);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }

        //Shoot a particle
        particle.shoot(robot.catapultMotor, robot.touchCat, this);

        //Attempt to load second particle
        robot.ballPickerMotor.setPower(.9);
        safeSleep(2500);
        robot.ballPickerMotor.setPower(-.9);
        safeSleep(700);
        robot.ballPickerMotor.setPower(0);

        //Hopefully shoot second particle
        particle.shoot(robot.catapultMotor, robot.touchCat, this);



        //robot.allStop();
        myDrive.allStop();
        robot.catapultMotor.setPower(0);
        robot.ballPickerMotor.setPower(0);

        while(opModeIsActive()) {
            idle();
        }

    }

    public int newHeading(int currentHeading, int turnHeading) {
        int tempHeading;

        tempHeading = currentHeading + turnHeading;

        if (tempHeading > 360)
            tempHeading = tempHeading - 360;

        if (tempHeading < 0)
            tempHeading = tempHeading + 360;

        return (tempHeading);


    }

    public void safeSleep (long duration) {
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - duration < currentTime) && opModeIsActive()) {
            //Kill some time
        }
    }
}







