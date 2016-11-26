package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DeviceManager;

import java.util.Map;

/**
 * Created by judenki on 11/19/16.
 */

public class KernelPanicPlatform {

    public DcMotor  leftMotorFront   = null;
    public DcMotor  rightMotorFront  = null;
    public DcMotor  leftMotorBack   = null;
    public DcMotor  rightMotorBack  = null;

    public void init(HardwareMap ahwMap) {


        //hwMap.dcMotor.put("right motor", new DcMotorImpl(, 1));
        //hwMap.dcMotor.put("left motor", leftMotor);
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
    }
}
