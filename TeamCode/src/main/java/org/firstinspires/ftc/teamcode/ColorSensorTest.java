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


@TeleOp(name="Check color sensor(s)", group="Configuration")
public class ColorSensorTest extends LinearOpMode {

    // Completely made up numbers
    public static final int COLOR_SENSOR_1_ADDRESS = 0x10;
    public static final int COLOR_SENSOR_2_ADDRESS = 0x11;

    List<ColorSensor> colorSensors = new ArrayList<ColorSensor>();
    //ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {

        //color = hardwareMap.colorSensor.get("dim");

        for( Map.Entry<String, ColorSensor> c : hardwareMap.colorSensor.entrySet() ) {
            ColorSensor sensor = c.getValue();
            if( c.getKey().equals("<device-name-1>") ) {
                sensor.setI2cAddress( new I2cAddr(0x10) );
            }
            else if( c.getKey().equals("<device-name-2>") ) {
                sensor.setI2cAddress( new I2cAddr(0x11) );
            }
            // ...
            colorSensors.add( sensor );
        }

        telemetry.addData( "Number of sensors: ", "%d", colorSensors.size() );
        telemetry.update();


        // wait for the start button to be pressed
        waitForStart();

        while(opModeIsActive()) {
            int i = 0;
            for( ColorSensor color : colorSensors ) {
                telemetry.addLine("Sensor " + i + " (" + String.format("0x%x", color.getI2cAddress().get8Bit()) + ")" );
                telemetry.addData("    Red   ", "%d", color.red());
                telemetry.addData("    Green ", "%d", color.green());
                telemetry.addData("    Blue  ", "%d", color.blue());
                i += 1;
            }
            telemetry.update();
            idle();
        }

    }
}