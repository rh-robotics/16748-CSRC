package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.HashMap;

/** Creates Objects to store hardware elements and initializers.
 * HardwareElement's are essentially HardwareDevices plus initializers. */
public class HardwareElement<T extends HardwareDevice> {
    /** Stores value that is expected to be in the hardware map.
     * Assumed to be both the entry and key values. */
    public String name;
    /** Stores type of device as a HardwareDevice Class. */
    public Class<T> type;
    /** Stores Hardware device from hardwareMap.
     * Use this to run commands (eg. HardwareElement.device.setPower(0.5)) */
    public HardwareDevice device;
    /** Hashmap to store element's initializers in feature:initValue format. */
    public HashMap<String, String> initializers = new HashMap<>();

    /** Constructs a HardwareElement.
     * @param deviceType The type of the device.
     * @param name Represents value of both Key and Entry.
     * @param hardwareMap The FTC SDK hardware map.
     * */
    public HardwareElement(Class<T> deviceType, HardwareMap hardwareMap, String name) {
        this.name = name;
        this.type = deviceType;
        this.device = hardwareMap.get(type, name);
        setDefaultInitializers(deviceType);
    }

    /** Constructs a HardwareElement.
     * @param deviceType The type of the device.
     * @param name Represents value of both Key and Entry.
     * @param hardwareMap The FTC SDK hardware map.
     * @param initializers: key:value,key:value
     *      eg. "setDirection:REVERSE,setZeroPowerBehavior:BRAKE"
     *      Only necessary to list non-default pairs. */
    public HardwareElement(Class<T> deviceType, HardwareMap hardwareMap,
                           String name, String initializers) {
        this.name = name;
        this.type = deviceType;
        this.device = hardwareMap.get(deviceType, name);
        setDefaultInitializers(deviceType);
        this.initializers = createHashMap(initializers);
    }

    /** Sets default initializers given device type.
    *  TODO: Update this method when implementing new initializers:
    *   Add default initializers for any new hardwareElements. Initializers hashmap can be left
    *   blank but must have a case in the conditionals to handle it and avoid RuntimeException. */
    private void setDefaultInitializers(Class<T> deviceType) {
        if (deviceType.equals(DcMotor.class) || deviceType.equals(DcMotorEx.class)) {
            initializers.put("setDirection", "FORWARD");
            initializers.put("setZeroPowerBehavior", "BRAKE");
            initializers.put("setMode", "RUN_USING_ENCODER");
        } else if (deviceType.equals(Servo.class)) {
            initializers.put("setDirection", "FORWARD");
        } else if (deviceType.equals(CRServo.class)) {
            initializers.put("setZeroPowerBehavior", "BRAKE");
        } else if (deviceType.equals(TouchSensor.class)) {

        } else if (deviceType.equals(DistanceSensor.class)) {

        } else if (deviceType.equals(ColorSensor.class)) {

        } else {
            throw new RuntimeException(deviceType.getName() + " has not been " +
                    "implemented with default initializers.");
        }
    }

    /** Takes in string formatted "key:value,key:value" and creates HashMap. */
    private HashMap<String, String> createHashMap(String input) {
        String[] pairs = input.split(",");
        HashMap<String, String> inputHashMap = new HashMap<>();

        // Return value.
        HashMap<String, String> initializerHashMap = new HashMap<>();

        for (String pair : pairs) {
            String[] keyValue = pair.split(":");
            inputHashMap.put(keyValue[0], keyValue[1]);
        }

        /* Iterates through default initializers, checking if the HashMap created by inputs already
         * includes the key. If so, it changes the value to the input-provided value. */
        for (String defaultInitKey : this.initializers.keySet()) {
            if (inputHashMap.containsKey(defaultInitKey)) {
                initializerHashMap.put(defaultInitKey, inputHashMap.get(defaultInitKey));
            }
        }

        /* Checks if any input initialization features are given that are not yet listed. */
        for (HashMap.Entry<String, String> inputInitPair : inputHashMap.entrySet()) {
            if (!initializers.containsKey(inputInitPair.getKey())) {
                throw new RuntimeException(inputInitPair.getKey() +
                        " is not currently listed as an initialization feature.");
            }
        }

        return initializerHashMap;
    }
}