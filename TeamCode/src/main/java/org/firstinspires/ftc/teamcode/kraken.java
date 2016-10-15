package org.firstinspires.ftc.teamcode;
/**
 * Created by judenki on 9/24/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


    /**
     * Authors - Nate, Jack, Carlos, and Helen
     * Substantial restructuring in Apr 2016, by Howard
     */
    public class kraken extends OpMode {
        public enum unloader_state {Center, Left, Right}

        DcMotor left_Drive;
        DcMotor right_Drive;
        DcMotor esca_Drive;
        DcMotor Whinch;

        Servo Unload;
        Servo Right_Climber_Releaser;
        Servo Left_Climber_Releaser;
        Servo whinch_pos;
        Servo Kraken;


        public double left_releaser = 0;
        public double right_releaser = 1;
        public int i = 0;
        public int j = 0;
        public static double whinch_position = .5;
        public static float prev_leftStickPos = 0;
        public static float prev_rightStickPos = 0;
        public static final double HI_FILT_CONST = 0.96;
        public static final double MID_FILT_CONST = 0.5;
        public static final double LOW_FILT_CONST = 0.1;
        public static double whinch_extend = 0;
        public static double w_effort = 0;
        public static double max_whinch_pos = .9;
        public static double max_whinch_effort = 1;
        public static double whinch_extend_time = 0;
        public static double whinch_retract_time = 0;
        public static double drive_power = 0;
        unloader_state cur_state;

        @Override
        public void init() {
            //get references to the drive motor from the hardware map
            left_Drive = hardwareMap.dcMotor.get("left_drive");
            right_Drive = hardwareMap.dcMotor.get("right_drive");

            Unload = hardwareMap.servo.get("Unload");

            //DcMotor whinch  = hardwareMap.dcmotor.get("whinch");
            Whinch = hardwareMap.dcMotor.get("whinch");

            whinch_pos = hardwareMap.servo.get("whinch_pos");

            //DcMotor esca_Drive = hardwareMap.dcMotor.get("esca_drive");
            esca_Drive = hardwareMap.dcMotor.get("esca_drive");

            Right_Climber_Releaser = hardwareMap.servo.get("Right_Climber_Releaser");
            Left_Climber_Releaser = hardwareMap.servo.get("Left_Climber_Releaser");
            Right_Climber_Releaser.setDirection(Servo.Direction.REVERSE);


            left_Drive.setDirection(DcMotor.Direction.REVERSE);

            Kraken = hardwareMap.servo.get("Kraken");

            max_whinch_pos = .9;


            cur_state = unloader_state.Center;

            // set the mode
            // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
            left_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

/************************************************************************/
/*                 climb_release                                        */
/*   Inputs: (gamepad2 x and y)                                         */
/*   Outputs: Left_Climber_Releaser servo                               */
/*            Right_Climber_Releaser servo                              */
/*   Returns:  Status                                                   */
        /************************************************************************/
        public int climb_release() {
       /* int climb_release_status = 0;
        double Right_Flip_pos = 0.45;
        double Left_Flip_pos = 0.45;

        if (gamepad2.x) {
            Right_Flip_pos = 1;
        }
        if (gamepad2.y) {
            Left_Flip_pos = 0;
        }
        Right_Climber_Releaser.setPosition(Right_Flip_pos);
        Left_Climber_Releaser.setPosition(Left_Flip_pos);                                                                                                                                                                                                                                                       *HI
        return climb_release_status;
    }*/
            if (gamepad2.x){
                if (right_releaser == 1){
                    i += 1;
                    if (i >= 100) {
                        right_releaser = 0;
                        i = 0;
                    }
                }
                else if (right_releaser == 0){
                    i+= 1;
                    if (i>= 100){
                        right_releaser = 1;
                    }
                }
            }
            else {
                i=0;
            }

            if (gamepad2.y){
                if (left_releaser == 1){
                    j += 1;
                    if (j >= 100) {
                        left_releaser = 0;
                        j = 0;
                    }
                }
                else if (left_releaser == 0){
                    j+= 1;
                    if (j>= 100){
                        left_releaser = 1;
                    }
                }
            }
            else {
                j = 0;
            }

            Left_Climber_Releaser.setPosition(left_releaser/2);
            Right_Climber_Releaser.setPosition(right_releaser/2);
            return 0;
        }

/************************************************************************/
/*                 unloader  -- Unload debris controls                  */
/*   Inputs: (gamepad2 left and right   dpad                            */
/*   Outputs: Unloader                                                  */
/*   Returns:  Status                                                   */
        /***********************************************************************/

        public int unloader() {
            int unloader_status = 0;
            double unload_target = .5;

            if (gamepad2.dpad_right) unload_target = 0;
            if (gamepad2.dpad_left) unload_target = 1;
            Unload.setPosition(unload_target);
            return unloader_status;
        }


/************************************************************************/
/*                     filter_input                                     */
/*      Read the inputs, post process them, and produce the             */
/*      output Motor Drive commands                                     */
/*   Inputs: stick position, previous stick position                    */
/*   Outputs: (return value)                                            */
/*   Returns:  Filtered stick position                                  */
        /************************************************************************/
        public static float filter_input(float instick, float prevStickPos) {
            float filter_constant = 0;  /*init to zero just in case */
        /* cube the input stick to give better response around zero */
            instick = instick * instick * instick;

        /*  Select which filter constant to use for stick position. Use -0.05 */
        /* as lowest stick pos for filtered value lets the controller springs */
        /* overshoot a little                                                 */
        /* filt const is applied to prev Stick pos in order to keep it from   */
        /* being jittery since raw stick position could be all over the place.*/
            if ((instick > -.05) && (instick < prevStickPos)) {
                if (prevStickPos > 0.85) {
                    filter_constant = (float) HI_FILT_CONST;
                    //filter_constant = (float)0.95;
                } else if (prevStickPos > 0.6) {
                    filter_constant = (float) MID_FILT_CONST;
                    //filter_constant = (float)0.65;
                } else if (prevStickPos > 0.4) {
                    filter_constant = (float) LOW_FILT_CONST;
                    //filter_constant = (float)0.35;
                }
            }
	/*  Basic formula for a simple 1st order filter is:      */
	/*      ((1-FC)* input) + (FC * prev_value)              */
	/*  where FC is filter constant ref: wikipedia, et. al.  */
	/* this works because we set the filter const to 0 for most conditions*/
	/* Only nonzero when decreasing and > some arbitrary value  */
            prevStickPos = ((1 - filter_constant) * instick) + (filter_constant * prevStickPos);
            return (prevStickPos);
        }

/************************************************************************/
/*                      escalator_control                               */
/*      Control the motor to raise or lower debris to the platform      */
/*   Inputs: (gamepad1 left and right triggers)                         */
/*   Outputs: Escalator Motor Drive commands                            */
/*   Returns: status of escalator_control                               */
        /************************************************************************/
        public int escalator_control() {
            int esca_status = 0;

            float right_trigger = gamepad1.right_trigger;
            float left_trigger = gamepad1.left_trigger;
            float esca_drive_effort = 0;

            if (right_trigger > .1) {
                esca_drive_effort = -1 * right_trigger;
            } else if (left_trigger > .1) {
                esca_drive_effort = left_trigger;
            }

            esca_Drive.setPower(esca_drive_effort);
            return (esca_status);
        }

/************************************************************************/
/*                      drive_motor_control                             */
/*      Read the inputs, post process them, and produce the             */
/*      output Motor Drive commands                                     */
/*   Inputs: whinch effort, whinch duration                             */
/*           (gamepad1 left stick y, gamepad1 right stick y)            */
/*   Outputs: Left motor Drive, Right Motor Drive commands              */
/*   Returns: Absolute Value of motor effort                            */
        /************************************************************************/

        public float drive_motor_control(double ret_cts) {
        /*float left_drive_effort = 0;*/
        /*float right_drive_effort = 0;*/
            float abs_effort;
            float max_drive_value = (float) 1.00;
            float min_drive_value = (float) -1.00;

        /* first get the raw effort from stick inputs then restrict as needed */
        /* Before setting hard limits, filter the stick input to reduce jitter*/
            float left_drive_effort = filter_input(gamepad1.left_stick_y, prev_leftStickPos);
            prev_leftStickPos = left_drive_effort;
            float right_drive_effort = filter_input(gamepad1.right_stick_y, prev_rightStickPos);
            prev_rightStickPos = right_drive_effort;

        /* put a min and max on whinch effort depending on       */
        /* 1) if the whinch is being retracted for quite a while */
        /* 2) if the "low range" button has been asserted       */
            if (gamepad1.right_bumper) {
                max_drive_value = (float) 0.3;
            }
            if (ret_cts > 15000) {
                min_drive_value = (float) 0.10;
                max_drive_value = (float) 0.25;
            }

            if (left_drive_effort < min_drive_value) left_drive_effort = min_drive_value;
            if (left_drive_effort > max_drive_value) left_drive_effort = max_drive_value;

            if (right_drive_effort < min_drive_value) right_drive_effort = min_drive_value;
            if (right_drive_effort > max_drive_value) right_drive_effort = max_drive_value;

            left_Drive.setPower(left_drive_effort);
            right_Drive.setPower(right_drive_effort);

        /* abs_effort restricts whinch retract speed while in motion       */
        /* tricky part is to keep it from interfering with drive commands */
            abs_effort = Math.abs(left_drive_effort) + Math.abs(right_drive_effort);
            return abs_effort;
        }

/************************************************************************/
/*                      whinch_extend_control                           */
/*      Control the motor to raise or lower debris to the platform      */
/*   Inputs: (gamepad2 a and b ), whinch_angle                          */
/*   Outputs: whinch Motor Drive commands, whinch_extend_time           */
/*   Returns: whinch_effort                                             */
        /************************************************************************/
        public double whinch_extend_control(double whinch_angle, double abs_effort) {

            max_whinch_effort = 1;
            whinch_extend = 0;
            if (gamepad2.b) {
                whinch_extend = .25;
                max_whinch_pos = .75;
                whinch_extend_time += 1;
            } else {
                whinch_extend_time = 0;
            }

            if (gamepad2.a) {
                whinch_extend = -.95;
                whinch_retract_time += 1;
            } else
            {whinch_retract_time = 0;}

        /*if (left_drive_effort >= 0.05) {
            if (right_drive_effort >= 0.05)*/
            if (abs_effort >= 0.5) max_whinch_effort = 0.4;
            if (whinch_extend > max_whinch_effort) whinch_extend = max_whinch_effort;

            Whinch.setPower(whinch_extend);
            return (whinch_extend);
        }
/************************************************************************/
/*                      whinch_angle_control                             */
/*      Control the motor to raise or lower debris to the platform      */
/*   Inputs: (gamepad2 left and right bumpers )                         */
/*   Outputs: whinch angle servo commands                                */
/*   Returns: whinch position                                            */

        /**
         * ********************************************************************
         */
        public double whinch_angle_control(double whinch_effort) {
            int whinch_extending = 0;
            int whinch_retracting = 0;

            max_whinch_effort = 1;
            if (whinch_effort < 0) whinch_extending = 1;
            if (gamepad2.right_bumper) {
                whinch_position = whinch_position + 0.001;
            } else if (gamepad2.left_bumper) {
                whinch_position = whinch_position - 0.001;
            }
            if ((whinch_extend_time >= 1000) && (whinch_extending == 1)) {
                if (whinch_position > 0.75) {
                    whinch_position = whinch_position - 0.01;
                }
            }
            if (whinch_position < 0) {
                whinch_position = 0;
            }
            if (whinch_position > max_whinch_pos) {
                whinch_position = max_whinch_pos;
            }

            whinch_pos.setPosition(whinch_position);
            return (whinch_position);

        }

        /************************************************************************/
/*        CAUTION: KRAKEN CODE
/*        Drops the climbers in the bin
/*
/*
*/
        public int kraken_unleash() {
            int kraken_unleash_status = 0;
            int operate_kraken = 1;
            double kraken_target = 1;

            if (gamepad1.y) {
                kraken_target = 0;
                operate_kraken = 1;
            }
            if (gamepad2.x) {
                kraken_target = 1;
                operate_kraken = 1;
            }
            if (operate_kraken == 1) {
                Kraken.setPosition(kraken_target);
            }
            return kraken_unleash_status;

        }
/************************************************************************/
/*                                  MAIN                                */
        /************************************************************************/
        @Override
        public void loop() {
// Need to add calls to the broken out methods
// also need to ensure the data goes between them smoothly
// and ... I dunno, do you think there'll be a debug cycle ?  :D


        /* whinch control  */
            double w_angle = whinch_angle_control(w_effort);
            w_effort = whinch_extend_control(w_angle, drive_power);

        /* Escalator control */
            int unload_status = unloader();
            int escalator_status = escalator_control();
            int climb_release_status = climb_release();
            int kraken_unleash_status = kraken_unleash();

        /* drive the robot */
            drive_power = drive_motor_control(whinch_retract_time);
            //Push Gamepad commands to console
            telemetry.addData("01", "Gamepad1 info" + gamepad1.toString());
            telemetry.addData("02", "Gamepad2 info" + gamepad2.toString());
            telemetry.addData("03", i);
            telemetry.addData("4" , j);
            telemetry.addData("5" , unload_status);
            telemetry.addData("6" , escalator_status);
            telemetry.addData("7" , climb_release_status);
            telemetry.addData("8" , kraken_unleash_status);
        }
    }

