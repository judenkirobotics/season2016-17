package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Map;

/**
 * Created by judenki on 11/26/16.
 */

public class JudenKiPlatform {
    public DcMotor  leftMotorFront   = null;
    public DcMotor  rightMotorFront  = null;
    public DcMotor  leftMotorBack   = null;
    public DcMotor  rightMotorBack  = null;
    public DcMotor  catapultMotor  = null;
    public ColorSensor colorTheBottom = null;
    public ColorSensor colorTheSide = null;
    public GyroSensor gyro = null;
    public TouchSensor touchCat     = null;
    public Servo beaconServo = null;
    //public DcMotor  ballPickerMotor = null;


    // For unknown reasons must convert the 8-bit address programmed by the MR tool to a
    // 7-bit address.  This is effectively a divide by two.
    public static final int COLOR_SENSOR_BOTTOM_ADDRESS = 0x3c>>1;
    public static final int COLOR_SENSOR_SIDE_ADDRESS = 0x3e>>1;

    public void init(HardwareMap ahwMap) {


        // Define and Initialize Motors
        System.out.println("Showing the dcMotor hwMap");
        for (Map.Entry<String, DcMotor> i : ahwMap.dcMotor.entrySet()) {
            System.out.println("motor key: " + i.getKey());
            System.out.println("motor value: " + i.getValue().toString());
        }
        System.out.println("Done showing the dcMotor hwMap");
        rightMotorFront = ahwMap.dcMotor.get("right motor front");
        leftMotorFront = ahwMap.dcMotor.get("left motor front");
        rightMotorBack = ahwMap.dcMotor.get("right motor back");
        leftMotorBack = ahwMap.dcMotor.get("left motor back");
        catapultMotor = ahwMap.dcMotor.get("catapult motor");
        //ballPickerMotor = ahwMap.dcMotor.get("ball picker motor");

        //configure 2 color sensors
       /* colorTheSide = ahwMap.colorSensor.get("color side");
        colorTheSide.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_SIDE_ADDRESS));
        colorTheSide.enableLed(false);
        colorTheBottom = ahwMap.colorSensor.get("color bottom");
        colorTheBottom.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_BOTTOM_ADDRESS));
        colorTheBottom.enableLed(true);
*/
        //Gyro Sensor
        gyro = ahwMap.gyroSensor.get("gyro");
        gyro.calibrate();
        while (gyro.isCalibrating()  == true) {
            // need to add break out
             touchCat= ahwMap.touchSensor.get("touchCat");
        }


    }
}
