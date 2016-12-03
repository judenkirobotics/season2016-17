package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.locks.Lock;

/**
 * This is a custom made program by Team 9915 to change the Modern Robotics Color Sensor I2C Address.
 * <p>MR Color Sensor has address: 0x3c
 * If you want to use multiple MR color sensors, you need to change the I2C address.
 * <p>
 *
 * In this program, the I2C address is changed to 0x42, you can use this program to change additional ones.
 *
 * @author: Team 9915
 *
 */
//@TeleOp(name="Rewrite color sensor address (0x3c) to 0x3f", group="Configuration")
public class I2CRewrite extends LinearOpMode {

    /*
    public static final int ADDRESS_SET_NEW_I2C_ADDRESS = 0x70;

    // trigger bytes used to change I2C address on ModernRobotics sensors.
    public static final byte TRIGGER_BYTE_1 = 0x55;
    public static final byte TRIGGER_BYTE_2 = (byte) 0xaa;

    public static final byte COLOR_SENSOR_ORIGINAL_ADDRESS = 0x3c;

    public static final int READ_MODE = 0x80;
    public static final int ADDRESS_MEMORY_START = 0x03;
    public static final int TOTAL_MEMORY_LENGTH = 0x06;

    public static final int BUFFER_CHANGE_ADDRESS_LENGTH = 0x03;

    int port = 3;

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    int currentAddress = COLOR_SENSOR_ORIGINAL_ADDRESS;
    // I2c addresses on Modern Robotics devices must be divisible by 2, and between 0x7e and 0x10
    // Different hardware may have different rules.
    // Be sure to read the requirements for the hardware you're using!
    int newAddress = 0x42;
    */

    // I2c addresses on Modern Robotics devices must be divisible by 2, and between 0x7e and 0x10
    // Different hardware may have different rules.
    // Be sure to read the requirements for the hardware you're using!
    int newAddress = 0x3f;

    DeviceInterfaceModule dim;

    ColorSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {

        sensor = hardwareMap.colorSensor.get("dim");

        // wait for the start button to be pressed
        waitForStart();

        sensor.setI2cAddress( new I2cAddr(newAddress) );

        telemetry.addLine( "New i2c address: " + sensor.getI2cAddress() );

        /*
        // set up the hardware devices we are going to use
        dim = hardwareMap.deviceInterfaceModule.get("dim");



        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);
        writeCache = dim.getI2cWriteCache(port);
        writeLock = dim.getI2cWriteCacheLock(port);

        // wait for the start button to be pressed
        waitForStart();

        performAction("read", port, currentAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);

        while(!dim.isI2cPortReady(port)) {
            telemetry.addData("I2cAddressChange", "waiting for the port to be ready...");
            sleep(1000);
        }

        // update the local cache
        dim.readI2cCacheFromController(port);

        // Enable writes to the correct segment of the memory map.
        performAction("write", port, currentAddress, ADDRESS_SET_NEW_I2C_ADDRESS, BUFFER_CHANGE_ADDRESS_LENGTH);

        waitOneFullHardwareCycle();

        // Write out the trigger bytes, and the new desired address.
        writeNewAddress();
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);

        telemetry.addData("I2cAddressChange", "Giving the hardware some time to make the change...");

        // Changing the I2C address takes some time.
        for (int i = 0; i < 5000; i++) {
            waitOneFullHardwareCycle();
            waitOneFullHardwareCycle();
        }

        // Query the new address and see if we can get the bytes we expect.
        dim.enableI2cReadMode(port, new I2cAddr(newAddress), ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);

        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);

        telemetry.addData("I2cAddressChange", "Successfully changed the I2C address." + String.format("New address: %02x", newAddress));
        */
    }

    /*
    private void performAction(String actionName, int port, int i2cAddress, int memAddress, int memLength) {
        if (actionName.equalsIgnoreCase("read")) dim.enableI2cReadMode(port, new I2cAddr(i2cAddress), memAddress, memLength);
        if (actionName.equalsIgnoreCase("write")) dim.enableI2cWriteMode(port, new I2cAddr(i2cAddress), memAddress, memLength);

        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);
        dim.readI2cCacheFromController(port);
    }

    private void writeNewAddress() {
        try {
            writeLock.lock();
            writeCache[4] = (byte) newAddress;
            writeCache[5] = TRIGGER_BYTE_1;
            writeCache[6] = TRIGGER_BYTE_2;
        } finally {
            writeLock.unlock();
        }
    }
    */
}