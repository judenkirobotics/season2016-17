package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;
import org.firstinspires.ftc.robotcontroller.external.samples.KernelPanicPlatform;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;
import org.firstinspires.ftc.robotcontroller.external.samples.KernelPanicPlatform;


/**
 * Created by Kernel Panic on 11/26/16.
 */
@Autonomous(name="Kernel Panic Autonomous", group="Kernel Panic")
public class KernelPanicAutonomous extends LinearOpMode {
    //Juden Ki Launching Robot
    KernelPanicPlatform robot = new KernelPanicPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //Clean this up because of drive class
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        waitForStart();

        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

    }
}
