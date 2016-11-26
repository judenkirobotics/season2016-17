package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;
import org.firstinspires.ftc.robotcontroller.external.samples.KernelPanicPlatform;

/**
 * Created by judenki on 11/19/16.
 */
@Autonomous(name="Square Drive Test", group="Kernel-Panic")
public class SquareDriveTest extends LinearOpMode {
    KernelPanicPlatform robot = new KernelPanicPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);

        // Move 36 inches
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }


        // Sleep 1 seconds
        //SystemClock.sleep(1000);

        // Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 3 seconds
        //SystemClock.sleep(1000);

        // Move 36 inches
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 1 second
        //SystemClock.sleep(1000);

        // Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 1 seconds
        //SystemClock.sleep(1000);

        // Move 36 inches
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 1 seconds
        //SystemClock.sleep(1000);

        // Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 1 seconds
        //SystemClock.sleep(1000);

        // Move 36 inches
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        // Sleep 1 Second
        //SystemClock.sleep(1000);

        // Turn 90 degrees
        myDrive.turn(90, 0.4);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        while(opModeIsActive()) {
            idle();
        }
    }
}
