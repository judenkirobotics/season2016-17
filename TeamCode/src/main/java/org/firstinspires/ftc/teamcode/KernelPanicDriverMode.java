package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Kernel Panic on 11/26/16.
 * Actuators and Sensors for Driver Control Mode
 */



@TeleOp(name="Kernel Panic Driver Mode", group="Kernel Panic")
public class KernelPanicDriverMode extends LinearOpMode {

    KernelPanicPlatform robot = new KernelPanicPlatform();
    //public HardwareMap hardwareMap = null; // will be set in OpModeManager.runActiveOpMode
    double continuous = 0.00;


    @Override
    public void runOpMode() throws InterruptedException {
        double forward;
        double drift;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            //right = -gamepad1.right_stick_y;

            //left = -gamepad1.left_stick_y;
            //myDrive.moveForward(12, left);
            //myDrive.drift(gamepad1.left_stick_x);
            forward = -gamepad1.left_stick_y;

            //Smooth movement but still ends up being more of a pivot than a drift
            //drift = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;

            //Try limiting it to 25% of total power
            if (forward < .25) {
                drift = gamepad1.left_stick_x;
            }
            else {
                drift = gamepad1.left_stick_x * 0.25;
            }
            myDrive.driveMove(forward, drift);

            // Actuate the servos.
            if (gamepad1.a) {
                robot.frontServo.setPosition(robot.frontServo.getPosition()+10);
            }
            if (gamepad1.b) {
                robot.frontServo.setPosition(robot.frontServo.getPosition()-10);
            }
            if (gamepad1.x) {
                robot.backServo.setPosition(robot.backServo.getPosition()+10);
            }
            if (gamepad1.y) {
                robot.backServo.setPosition(robot.backServo.getPosition()-10);
            }



            // Send telemetry message to signify robot running;

            //telemetry.addData("left y",  "%.2f", left);
            //telemetry.addData("right y", "%.2f", right);
            telemetry.addData("left x",  "%.2f", gamepad1.left_stick_x);
            telemetry.addData("right x", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("heading", robot.gyro.getHeading());
            telemetry.addData("Bottom    Red   ", "%d", robot.colorBottom.red());
            telemetry.addData("Bottom    Green ", "%d", robot.colorBottom.green());
            telemetry.addData("Bottom    Blue  ", "%d", robot.colorBottom.blue());
            telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
            telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
            telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
          //  telemetry.addData("Range   ", robot.rangeReader.getReadWindow());




            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    private ElapsedTime period  = new ElapsedTime();
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}