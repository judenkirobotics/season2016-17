package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by judenki on 11/26/16.
 */






/**
 * Created by Kernel Panic on 11/26/16.
 * Actuators and Sensors for Driver Control Mode
 */

@TeleOp(name="juden Ki Driver Mode", group="Juden Ki")
public class JudenKiDriverMode extends LinearOpMode {

    public JudenKiPlatform robot = new JudenKiPlatform();
    //public HardwareMap hardwareMap = null; // will be set in OpModeManager.runActiveOpMode


    @Override
    public void runOpMode() throws InterruptedException {
        double forward;
        double drift;


        /*
         * Probably want to move this to platform class as both autonomous and driver use it.
        */
        robot.init(hardwareMap);

        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Drivers");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //
        robot.catapultMotor.setPower(.0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            forward = gamepad1.left_stick_y;

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

            //Actuate Catapult

            if (gamepad1.a)
                shootTheBall(robot.catapultMotor, robot.touchCat);
            if (gamepad1.b)
                robot.catapultMotor.setPower(.0);


            if ((gamepad1.right_trigger > 0.1) && (gamepad1.left_trigger < 0.1))
                robot.ballPickerMotor.setPower(.9);
            else if ((gamepad1.left_trigger > 0.1) && (gamepad1.right_trigger < 0.1))
                robot.ballPickerMotor.setPower(-0.9);
            else
                robot.ballPickerMotor.setPower(0);



            //Button Pusher Motor
            if (gamepad1.right_bumper) {
                robot.beaconServo.setPosition(1);
            }
            if (gamepad1.left_bumper) {
                robot.beaconServo.setPosition(0);

            }


            // Send telemetry message to signify robot running;
            telemetry.addData("left x",  "%.2f", gamepad1.left_stick_x);
            telemetry.addData("right x", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("heading", robot.gyro.getHeading());
/*            telemetry.addData("Bottom    Red   ", "%d", robot.colorTheBottom.red());
            telemetry.addData("Bottom    Green ", "%d", robot.colorTheBottom.green());
            telemetry.addData("Bottom    Blue  ", "%d", robot.colorTheBottom.blue());
            telemetry.addData("Side      Red   ", "%d", robot.colorTheSide.red());
            telemetry.addData("Side      Green ", "%d", robot.colorTheSide.green());
            telemetry.addData("Side      Blue  ", "%d", robot.colorTheSide.blue());
*/
            telemetry.addData("Catapult Fried  Dill  Pickle  touch ", robot.touchCat.isPressed()    );
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

    public void shootTheBall (DcMotor mot, TouchSensor touch)  {


        mot.setPower(1.0);

        //Wait some time for it to cycle past the touch sensor
        double currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentTime < 500) && opModeIsActive()) {
            //Kill some time
        }
        mot.setPower(.40);
        while ((touch.isPressed() != true) && opModeIsActive()) {
            //kill some time
        }
        mot.setPower(0);


    }

}