package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Creates Objects to store hardware elements and initializers.
 * HardwareElement's are essentially HardwareDevices plus initializers. */
public class HardwareElement {
    /** Stores value that is expected to be in the hardware map.
     * Expected to be the same as the object name in init conditionals. */
    public String entry;
    public java.lang.Class<HardwareDevice> type;
    public HardwareDevice device;
    public HashMap<String, String> initializers = new HashMap<>();

    /** Constructs a HardwareElement.
     * @param deviceType The type of the device.
     * @param entry The value that is expected to be in the hardware map.
     *      Assumes Key and Entry are the same.
     * @param hardwareMap The FTC SDK hardware map.
     * */
    public HardwareElement(HardwareMap hardwareMap, java.lang.Class<HardwareDevice> deviceType, String entry) {
        this.entry = entry;
        this.device = hardwareMap.get(type, entry);
        this.type = deviceType;
        setDefaultInitializers(deviceType);
    }

    /** /** Constructs a HardwareElement.
     * @param deviceType The type of the device.
     * @param entry The value that is expected to be in the hardware map.
     *      Assumes Key and Entry are the same.
     * @param hardwareMap The FTC SDK hardware map.
     * @param initializers: key:value,key:value
     *      eg. "setDirection:REVERSE,setZeroPowerBehavior:BRAKE"
     *      Only necessary to list non-default pairs. */
    public HardwareElement(HardwareMap hardwareMap, java.lang.Class<HardwareDevice> deviceType, String entry, String initializers) {
        this.entry = entry;
        this.device = hardwareMap.get(deviceType, entry);
        this.type = deviceType;
        this.initializers = createHashMap(initializers);
    }

    /** Sets default initializers given device type.
     * Update this method when implementing new initializers. **/
    private void setDefaultInitializers(java.lang.Class<HardwareDevice> deviceType) {
        if (deviceType.getName().equals("DcMotor")) {
            initializers.put("setDirection", "FORWARD");
            initializers.put("setZeroPowerBehavior", "BRAKE");
            initializers.put("setMode", "RUN_USING_ENCODER");
        } else if (deviceType.getName().equals("DcMotorEx")) {
            initializers.put("setDirection", "FORWARD");
            initializers.put("setZeroPowerBehavior", "BRAKE");
            initializers.put("setMode", "RUN_USING_ENCODER");
        } else {
            throw new RuntimeException(deviceType.getName() + " has not been " +
                    "implemented with default initializers.");
        }
    }

    /** Takes in string formatted "key:value,key:value" and creates HashMap. */
    private HashMap<String,String> createHashMap(String input) {
        String[] pairs = input.split(",");
        HashMap<String, String> inputHashMap = new HashMap<String, String>();

        for (int i = 0; i < pairs.length; i++) {
            inputHashMap.put(pairs[i].split(":")[0], pairs[i].split(":")[1]);
        }

        /* return value */
        HashMap<String, String> initializerHashMap = new HashMap<String,String>();

        /* Iterates through default initializers, checking if the HashMap created by inputs already
         * includes the key. If so, it changes the value to the input-provided value. */
        for (HashMap.Entry<String, String> defaultInitPair : initializers.entrySet()) {
            if(inputHashMap.containsKey(defaultInitPair.getKey())) {
                initializers.put(defaultInitPair.getKey(),
                        initializerHashMap.get(defaultInitPair.getKey()));
            } else {
                initializerHashMap.put(defaultInitPair.getKey(), defaultInitPair.getValue());
            }
        }

        /** Checks if any input initialization features are given that are not yet listed. */
        for (HashMap.Entry<String, String> inputInitPair : inputHashMap.entrySet()) {
            if(!initializers.containsKey(inputInitPair.getKey())){
                throw new RuntimeException(inputInitPair.getKey() +
                        " is not currently listed as an initialization feature.");
            }
        }

        return initializerHashMap;
    }
}