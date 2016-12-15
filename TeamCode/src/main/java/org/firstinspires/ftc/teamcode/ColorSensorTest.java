package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//@TeleOp(name="Check color sensor(s)", group="Configuration")
public class ColorSensorTest extends LinearOpMode {

    /*********************************************************************
     * THIS WORKS!!!!  Do not change a thing, put in platform, driver, and
     * autonomous code
     *********************************************************************/



    // For unknown reasons must convert the 8-bit address programmed by the MR tool to a
    // 7-bit address.  This is effectively a divide by two.
    public static final int COLOR_SENSOR_BOTTOM_ADDRESS = 0x3c>>1;
    public static final int COLOR_SENSOR_SIDE_ADDRESS = 0x3e>>1;

    ColorSensor bottomSensor = null;
    ColorSensor sideSensor   = null;

    @Override
    public void runOpMode() throws InterruptedException {


        bottomSensor = hardwareMap.colorSensor.get("color bottom");
        bottomSensor.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_BOTTOM_ADDRESS));
        bottomSensor.enableLed(true);
        sideSensor = hardwareMap.colorSensor.get("color side");
        sideSensor.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_SIDE_ADDRESS));
        sideSensor.enableLed(false);

        // wait for the start button to be pressed
        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Bottom Address - ", COLOR_SENSOR_BOTTOM_ADDRESS);
            telemetry.addData("Bottom    Red   ", "%d", bottomSensor.red());
            telemetry.addData("Bottom    Green ", "%d", bottomSensor.green());
            telemetry.addData("Bottom    Blue  ", "%d", bottomSensor.blue());
            telemetry.addData("Side   Address - ", COLOR_SENSOR_SIDE_ADDRESS);
            telemetry.addData("Side      Red   ", "%d", sideSensor.red());
            telemetry.addData("Side      Green ", "%d", sideSensor.green());
            telemetry.addData("Side      Blue  ", "%d", sideSensor.blue());

            telemetry.update();
            idle();
        }

    }
}