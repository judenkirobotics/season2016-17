
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


// Move forward some
        myDrive.moveForward(3 , -0.6 );
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }


        currentHeading = robot.gyro.getHeading();
        nextHeading = newHeading(currentHeading, -34);
        myDrive.driveMove(0,-0.5);
        while(robot.gyro.getHeading() > nextHeading) {
            //Kill some time
        }
        myDrive.allStop();

        // Move backwards  some
        myDrive.moveForward(4.5 , 0.6 );
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }


        shootTheBall(robot.catapultMotor,robot.touchCat);

        SystemClock.sleep(1000);

        shootTheBall(robot.catapultMotor,robot.touchCat);


        myDrive.moveForward(55,-0.6);
        while (myDrive.motorsRunning() == true) {
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

    public void shootTheBall (DcMotor mot, TouchSensor touch)  {


        mot.setPower(.90);

        //Wait some time for it to cycle past the touch sensor
        double currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - currentTime < 500) {
            //wait to have the launcher move off the touch sensor
        }

        while (touch.isPressed() != true) {
            //kill some time
        }
        mot.setPower(0);


    }
}



