package org.firstinspires.ftc.teamcode.subsystems;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
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
    private HashMap<String, HardwareElement> hardwareElements;

    /** Object that contains hardwareElements HashMap. */
    public Hardware() {
        this.hardwareElements = new HashMap<>();
    }

    /** Adds hardwareElement to hardwareElements arrayList in entry:hardwareElement format.*/
    public void introduce(HardwareElement hardwareElement){
        this.hardwareElements.put(hardwareElement.name, hardwareElement);
    }

    public void init(){
        for (int i = 0; i < this.hardwareElements.size(); i++) {
            HardwareElement currentElement = this.hardwareElements.get(i);
            java.lang.Class<HardwareDevice> hardwareType = currentElement.type;
            HashMap<String,String> initsHashMap = currentElement.initializers;

            /* Initializers should be added as needed to the conditional with their corresponding class. */
            if (hardwareType.getName().equals("DcMotor")) {
                /* Casting a separate deviceDcMotor object as a DcMotor type allows us
                 * run type-specific methods. (eg. deviceDcMotor.setDirection()) */
                DcMotor deviceDcMotor = (DcMotor) currentElement.device;

                if (initsHashMap.get("setDirection").equals("FORWARD")) {
                    deviceDcMotor.setDirection(DcMotor.Direction.FORWARD);
                } else if (initsHashMap.get("setDirection").equals("REVERSE")) {
                    deviceDcMotor.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    initValueRuntimeException("setDirection", initsHashMap.get("setDirection"));
                }

                if (initsHashMap.get("setZeroPowerBehavior").equals("BRAKE")) {
                    deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else if (initsHashMap.get("setZeroPowerBehavior").equals("FLOAT")) {
                    deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    initValueRuntimeException("setZeroPowerBehavior",
                            initsHashMap.get("setZeroPowerBehavior"));
                }

                if (initsHashMap.get("setMode").equals("RUN_USING_ENCODER")) {
                    deviceDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    initValueRuntimeException("setMode", initsHashMap.get("setMode"));
                }
            } else if (hardwareType.getName().equals("DcMotorEx")) {
                DcMotorEx deviceDcMotorEx = (DcMotorEx) currentElement.device;

                if (initsHashMap.get("setDirection").equals("FORWARD")) {
                    deviceDcMotorEx.setDirection(DcMotorEx.Direction.FORWARD);
                } else if (initsHashMap.get("setDirection").equals("REVERSE")) {
                    deviceDcMotorEx.setDirection(DcMotorEx.Direction.REVERSE);
                } else {
                    initValueRuntimeException("setDirection", initsHashMap.get("setDirection"));
                }

                if (initsHashMap.get("setZeroPowerBehavior").equals("BRAKE")) {
                    deviceDcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                } else if (initsHashMap.get("setZeroPowerBehavior").equals("FLOAT")) {
                    deviceDcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                } else {
                    initValueRuntimeException("setZeroPowerBehavior",
                            initsHashMap.get("setZeroPowerBehavior"));
                }

                if (initsHashMap.get("setMode").equals("RUN_USING_ENCODER")) {
                    deviceDcMotorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    initValueRuntimeException("setMode", initsHashMap.get("setMode"));
                }
            } else {
                throw new RuntimeException("Unimplemented hardware element: " +
                        hardwareType.getName());
            }
        }
    }

    private static void initValueRuntimeException(String key, String value) {
        throw new RuntimeException("Unimplemented hardware element method '" +
                key + ":" + value + "' encountered.");
    }
}