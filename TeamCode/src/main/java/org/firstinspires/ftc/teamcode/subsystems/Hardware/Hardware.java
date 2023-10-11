package org.firstinspires.ftc.teamcode.subsystems.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashMap;

/** eg. initialization code
 *
 * // class declaration
 * public Hardware robot;
 *
 * // in init
 * robot = new Hardware();
 * robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "leftFront"));
 * robot.init();
 *
 * // in loop
 * robot.HardwareElements.get("leftFront").device.setPower(0.5);
 */

public class Hardware {

    /** HashMap to hold all hardware elements in name/entry:hardwareElement format. */
    public HashMap<String, HardwareDevice> hardwareElements;

    /** Object that contains hardwareElements HashMap. */
    public Hardware() {
        this.hardwareElements = new HashMap<>();
    }

    /** Adds hardwareElement to hardwareElements arrayList in entry:hardwareElement format.*/
    public void introduce(HardwareElement hardwareElement){
        init(hardwareElement);
        this.hardwareElements.put(hardwareElement.name, hardwareElement.device);
    }

    private void init(HardwareElement hardwareElement){
        java.lang.Class<HardwareDevice> hardwareType = hardwareElement.type;
        HashMap<String,String> initsHashMap = hardwareElement.initializers;

        /* Initializers should be added as needed to the conditional with their corresponding class. */
        if (hardwareType.isInstance(DcMotor.class)) {
            /* Casting a separate deviceDcMotor object as a DcMotor type allows us
             * run type-specific methods. (eg. deviceDcMotor.setDirection()) */
            DcMotor deviceDcMotor = (DcMotor) hardwareElement.device;

            if (initsHashMap.get("setDirection").equals("FORWARD")) {
                deviceDcMotor.setDirection(DcMotor.Direction.FORWARD);
            } else if (initsHashMap.get("setDirection").equals("REVERSE")) {
                deviceDcMotor.setDirection(DcMotor.Direction.REVERSE);
            } else {
                throw new HardwareValueException("setDirection", initsHashMap.get("setDirection"));
            }

            if (initsHashMap.get("setZeroPowerBehavior").equals("BRAKE")) {
                deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (initsHashMap.get("setZeroPowerBehavior").equals("FLOAT")) {
                deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                throw new HardwareValueException("setZeroPowerBehavior",
                        initsHashMap.get("setZeroPowerBehavior"));
            }

            if (initsHashMap.get("setMode").equals("RUN_USING_ENCODER")) {
                deviceDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                throw new HardwareValueException("setMode", initsHashMap.get("setMode"));
            }
        } else if (hardwareType.isInstance(DcMotorEx.class)) {
            DcMotorEx deviceDcMotorEx = (DcMotorEx) hardwareElement.device;

            if (initsHashMap.get("setDirection").equals("FORWARD")) {
                deviceDcMotorEx.setDirection(DcMotorEx.Direction.FORWARD);
            } else if (initsHashMap.get("setDirection").equals("REVERSE")) {
                deviceDcMotorEx.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                throw new HardwareValueException("setDirection",
                        initsHashMap.get("setDirection"));
            }

            if (initsHashMap.get("setZeroPowerBehavior").equals("BRAKE")) {
                deviceDcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else if (initsHashMap.get("setZeroPowerBehavior").equals("FLOAT")) {
                deviceDcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            } else {
                throw new HardwareValueException("setZeroPowerBehavior",
                        initsHashMap.get("setZeroPowerBehavior"));
            }

            if (initsHashMap.get("setMode").equals("RUN_USING_ENCODER")) {
                deviceDcMotorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            } else {
                throw new HardwareValueException("setMode", initsHashMap.get("setMode"));
            }
        } else {
            throw new RuntimeException("Unimplemented hardware element: " +
                    hardwareType.getName());
        }
    }
}