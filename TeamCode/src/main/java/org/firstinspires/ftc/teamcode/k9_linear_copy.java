/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**
 * Hardware map (last updated: 2016/10/22)
 *
 */

@TeleOp(name="K9bot: Telop Tank", group="K9bot")
public class k9_linear_copy extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareK9bot   robot           = new HardwareK9bot();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    double          continuous      = 0.00;
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);


            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                armPosition += ARM_SPEED;
            else if (gamepad1.y)
                armPosition -= ARM_SPEED;

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;

            //Use A & B on Gamepad 2 to move continuous
            if (gamepad2.a)
                continuous ++;
            else if (gamepad2.b)
                continuous --;

            if (continuous > 256)
                continuous = 256;
            if (continuous < 0)
                continuous = 0;

            robot.continuous.setPower(continuous);

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);




            //
            //  Code for linear interpolation of optical distance sensor.
            // This should be wrapped in its own class at a later date
            //
            double opticalDistance[] = new double[9];
            opticalDistance[0]   =   0.996;   // Index into array is also the inches value
            opticalDistance[1]   =   0.20015;
            opticalDistance[2]   =   0.071404;
            opticalDistance[3]   =   0.02737;
            opticalDistance[4]   =   0.014663;
            opticalDistance[5]   =   0.01075;
            opticalDistance[6]   =   0.006843;
            opticalDistance[7]   =   0.004888;
            opticalDistance[8]   =   0.002933;

            double realDistance   = 0;


            for(int i=0; i<opticalDistance.length; i++) {
                if (robot.ods.getLightDetected() > opticalDistance[i]) {
                    double delta;

                    delta = (opticalDistance [i-1] - opticalDistance[i]) / opticalDistance[i];
                    realDistance = i - delta;  // Course interpolation.  May want to make it better.
                    break;
                }
            }




            //Test code for touch Sensor, have it spin sail (CRServo) servo
            //if (robot.touch.isPressed())
            //    robot.continuous.setPower(0);
            //else
            //    robot.continuous.setPower(1);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("cont",  "%.2f", continuous);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("left  encoder ", "%d", robot.leftMotor.getCurrentPosition());
            telemetry.addData("right encoder ", "%d", robot.rightMotor.getCurrentPosition());
            telemetry.addData("arm port   ",  "%d", robot.arm.getPortNumber());
            telemetry.addData("claw port  ",  "%d", robot.claw.getPortNumber());
            telemetry.addData("cont port  ", "%d", robot.continuous.getPortNumber());
            telemetry.addData("touch sensor  ", "%b", robot.touch.isPressed());
            telemetry.addData("seeker angle ", "%.4f", robot.seeker.getAngle());
            telemetry.addData("seeker distance ", "%.4f", robot.seeker.getStrength());

            telemetry.addData("Color Red   ", "%d", robot.color.red());
            telemetry.addData("Color Green ", "%d", robot.color.green());
            telemetry.addData("Color Blue  ", "%d", robot.color.blue());

            telemetry.addData("Distance   ", "%.6f", robot.ods.getLightDetected());
            telemetry.addData("Real Distance ", "%1.2f", realDistance);

            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
