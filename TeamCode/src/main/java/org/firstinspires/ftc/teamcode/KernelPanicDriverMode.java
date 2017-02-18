package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Kernel Panic on 11/26/16.
 * Actuators and Sensors for Driver Control Mode
 */



//@TeleOp(name="Kernel Panic Driver Mode", group="Kernel Panic")
public class KernelPanicDriverMode extends LinearOpMode {

    KernelPanicPlatform robot = new KernelPanicPlatform();
    //public HardwareMap hardwareMap = null; // will be set in OpModeManager.runActiveOpMode
    double continuous = 0.00;
    double prevLeftY = 0.0;
    double prevLeftX = 0.0;
    
/************************************************************************/
/*                     filter_input                                     */
/*      Read the inputs, post process them, and produce the             */
/*      output Motor Drive commands                                     */
/*   Inputs: stick position, previous stick position                    */
/*   Outputs: (return value)                                            */
/*   Returns:  Filtered stick position                                  */
/************************************************************************/
    public static double filter_input(double instick, double prevStickPos) {
        double filter_constant = 0;  /*init to zero just in case */
        /* cube the input stick to give better response around zero */
        instick = instick * instick * instick;

        if (prevStickPos > 0.85) {
            filter_constant = 0.35;
        } else if (prevStickPos > 0.6) {
            filter_constant = 0.45;
        } else if (prevStickPos > 0.4) {
            filter_constant = 0.65;
        }
        prevStickPos = ((1 - filter_constant) * instick) + (filter_constant * prevStickPos);
        return (prevStickPos);
    }


    public void runOpMode() throws InterruptedException {
        double forward;
        double drift;
        boolean triggerPressed = false;

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
            forward = filter_input(-gamepad1.left_stick_y, prevLeftY);
            prevLeftY = forward;
            //Smooth movement but still ends up being more of a pivot than a drift
            //drift = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;

            //Try limiting it to 25% of total power
            if (forward < .25) {
                drift = filter_input(gamepad1.left_stick_x, prevLeftX);
            }
            else {
                drift = gamepad1.left_stick_x * 0.25;
            }
            prevLeftX = drift;
            myDrive.driveMove(forward, drift);

            // Actuate the servos.
            if (gamepad1.a) {
                robot.frontServo.setDirection(CRServo.Direction.FORWARD);
                robot.frontServo.setPower(robot.SERVO_EXTEND_POWER);
            }
            else if (gamepad1.b) {
                robot.frontServo.setDirection(CRServo.Direction.REVERSE);
                robot.frontServo.setPower(robot.SERVO_RETRACT_POWER);
            }
            else
                robot.frontServo.setPower(robot.SERVO_STOP_POWER);


            if (gamepad1.x) {
                robot.backServo.setDirection(CRServo.Direction.FORWARD);
                robot.backServo.setPower(robot.SERVO_EXTEND_POWER);
            }
            else if (gamepad1.y) {
                robot.backServo.setDirection(CRServo.Direction.REVERSE);
                robot.backServo.setPower(robot.SERVO_RETRACT_POWER);
            }
            else
                robot.backServo.setPower(robot.SERVO_STOP_POWER);

            if(gamepad1.right_bumper && triggerPressed) {
                robot.ballLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.ballLiftMotor.setPower(1);
            }
            else if(gamepad1.left_bumper && triggerPressed) {
                robot.ballLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.ballLiftMotor.setPower(1);
            }
            else {
                robot.ballLiftMotor.setPower(0);
            }


            if(gamepad1.right_trigger != 0.0)  {
                robot.liftServoLeft.setDirection(Servo.Direction.FORWARD);
                robot.liftServoLeft.setPosition(1);
                robot.liftServoRight.setDirection(Servo.Direction.FORWARD);
                robot.liftServoRight.setPosition(0);
                triggerPressed = false;
                telemetry.addData("Open the ARMS",1);
            }
            else if(gamepad1.left_trigger != 0.0) {
                robot.liftServoLeft.setDirection(Servo.Direction.FORWARD);
                robot.liftServoLeft.setPosition(0);
                robot.liftServoRight.setDirection(Servo.Direction.FORWARD);
                robot.liftServoRight.setPosition(1);
                triggerPressed = true;
                telemetry.addData("Close the ARMS", 1);
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
            //telemetry.addData("Voltage", "%d", robot.voltageSensor.getVoltage());
            //  telemetry.addData("Range   ", robot.rangeReader.getReadWindow());




            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            //waitForTick(40);
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
