package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;

public class HardwareElement {
    /** The type of the device. */
    private final java.lang.Class type;

    /** The key that can be accessed from the Hardware class. */
    private final String key;
    /** The value that is expected to be in the hardware map. */
    private final String entry;
     /** Key: The initializing feature being referred to.
     * Value: The value the initializing feature will be set to. */
    private HashMap<String, String> initializers = new HashMap<String, String>();

    /** Hashmap with default pairs of initializingFeature:defaultValue.*/
    public static HashMap<String, String> DEFAULT_INITIALIZERS = new HashMap<String,String>() {{
            DEFAULT_INITIALIZERS.put("setDirection", "FORWARD");
            DEFAULT_INITIALIZERS.put("setZeroPowerBehavior", "BRAKE");
            DEFAULT_INITIALIZERS.put("setMode", "RUN_USING_ENCODER");
        }};

    /** Constructs a HardwareElement.
     * @param type The type of the device.
     * @param key The key that can be accessed from the Hardware class.
     * @param entry The value that is expected to be in the hardware map. */
    public HardwareElement(java.lang.Class type, String key, String entry) {
        this.type = type;
        this.key = key;
        this.entry = entry;
        this.initializers = DEFAULT_INITIALIZERS;
    }

    /** Format initializers: key:value,key:value
     * eg. "setDirection:REVERSE,setZeroPowerBehavior:BRAKE"
     * ONLY NECESSARY TO LIST NON-DEFAULT PAIRS. */
    public HardwareElement(java.lang.Class type, String key, String entry, String initializers) {
        this.type = type;
        this.key = key;
        this.entry = entry;
        this.initializers = createHashMap(initializers);
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
         * includes the key. If so, it adds that key:value pair to initializerHashMap.
         * If not, it adds the default key:value pair. */
        for (HashMap.Entry<String, String> defaultInitPair : DEFAULT_INITIALIZERS.entrySet()) {
            if(inputHashMap.containsKey(defaultInitPair.getKey())) {
                initializerHashMap.put(defaultInitPair.getKey(),
                        initializerHashMap.get(defaultInitPair.getKey()));
            } else {
                initializerHashMap.put(defaultInitPair.getKey(), defaultInitPair.getValue());
            }
        }

        /** Checks if any input initialization features are given that are not yet listed. */
        for (HashMap.Entry<String, String> inputInitPair : inputHashMap.entrySet()) {
            if(!DEFAULT_INITIALIZERS.containsKey(inputInitPair.getKey())){
                throw new RuntimeException(inputInitPair.getKey() +
                        " is not currently listed as an initialization feature.");
            }
        }

        return initializerHashMap;
    }
}