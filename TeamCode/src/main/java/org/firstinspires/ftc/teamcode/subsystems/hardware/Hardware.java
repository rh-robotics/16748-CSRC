package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Objects;

/** eg. initialization code
 * <br/>
 * // class declaration
 * public Hardware robot;
 * <br/>
 * // in init
 * robot = new Hardware();
 * robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "leftFront"));
 * robot.init();
 * <br/>
 * // in loop
 * robot.HardwareElements.get("leftFront").device.setPower(0.5);
 */

public class Hardware {
    /** A reference to a HardwareMap created in an OpMode. */
    private final HardwareMap hardwareMap;

    /** A reference to a Telemetry created in an OpMode. */
    private final Telemetry telemetry;

    /** HashMap to hold all hardware elements in name/entry:hardwareElement format. */
    public HashMap<String, HardwareDevice> hardwareElements = new HashMap<>();

    /** Object that contains hardwareElements HashMap. */
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /** Adds hardwareElement to hardwareElements arrayList in entry:hardwareElement format.*/
    public void introduce(HardwareElement<? extends HardwareDevice> hardwareElement){
        init(hardwareElement);
        this.hardwareElements.put(hardwareElement.name, hardwareElement.device);
    }

    private void init(HardwareElement<?> hardwareElement) {
        /* Initializers should be added as needed to the conditional with their corresponding
         * class. */
        if (hardwareElement.device instanceof DcMotor) {
            initDcMotor(hardwareElement, hardwareElement.initializers);
        } else if (hardwareElement.device instanceof CRServo) {
            initServo(hardwareElement, hardwareElement.initializers);
        } else if (hardwareElement.device instanceof TouchSensor) {
            initTouchSensor(hardwareElement, hardwareElement.initializers);
        } else if (hardwareElement.device instanceof ColorSensor) {
            initColorSensor(hardwareElement, hardwareElement.initializers);
        } else {
            throw new RuntimeException("Unimplemented hardware element of type '" +
                    hardwareElement.getClass().getSimpleName() + "'.");
        }
    }

    private void initDcMotor(HardwareElement<?> hardwareElement,
                             HashMap<String, String> initsHashMap) {
        /* Casting a separate deviceDcMotor object as a DcMotor type allows us
         * run type-specific methods. (eg. deviceDcMotor.setDirection()) */
        DcMotor deviceDcMotor = (DcMotor) hardwareElement.device;

        if (initsHashMap.containsKey("setDirection")) {
            switch (Objects.requireNonNull(initsHashMap.get("setDirection"))) {
                case "FORWARD":
                    deviceDcMotor.setDirection(DcMotor.Direction.FORWARD);
                    break;
                case "REVERSE":
                    deviceDcMotor.setDirection(DcMotor.Direction.REVERSE);
                    break;
                default:
                    throw new HardwareValueException("setDirection", initsHashMap.get("setDirection"));
            }
        }

        if (initsHashMap.containsKey("setZeroPowerBehavior")) {
            switch (Objects.requireNonNull(initsHashMap.get("setZeroPowerBehavior"))) {
                case "BRAKE":
                    deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    break;
                case "FLOAT":
                    deviceDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                default:
                    throw new HardwareValueException("setZeroPowerBehavior",
                            initsHashMap.get("setZeroPowerBehavior"));
            }
        }

        if (initsHashMap.containsKey("setMode")) {
            if (Objects.requireNonNull(initsHashMap.get("setMode")).equals("RUN_USING_ENCODER")) {
                deviceDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                throw new HardwareValueException("setMode", initsHashMap.get("setMode"));
            }
        }
    }
    private void initServo(HardwareElement<?> hardwareElement,
                           HashMap<String, String> initsHashMap) {
        /* Casting a separate deviceDcMotor object as a DcMotor type allows us
         * run type-specific methods. (eg. deviceDcMotor.setDirection()) */
        CRServo deviceCRServo = (CRServo) hardwareElement.device;

        if (initsHashMap.containsKey("setDirection")) {
            switch (Objects.requireNonNull(initsHashMap.get("setDirection"))) {
                case "FORWARD":
                    deviceCRServo.setDirection(CRServo.Direction.FORWARD);
                    break;
                case "REVERSE":
                    deviceCRServo.setDirection(CRServo.Direction.REVERSE);
                    break;
                default:
                    throw new HardwareValueException("setDirection", initsHashMap.get("setDirection"));
            }
        }
    }

    private void initTouchSensor(HardwareElement<?> hardwareElement,
                                 HashMap<String, String> initsHashMap) {
        if (!initsHashMap.isEmpty()) {
            throw new RuntimeException("Touch Sensor unnecessary initializers included.");
        }

        TouchSensor deviceTouchSensor = (TouchSensor) hardwareElement.device;

    }

    private void initColorSensor(HardwareElement<?> hardwareElement,
                                 HashMap<String, String> initsHashMap) {
        ColorSensor deviceColorSensor = (ColorSensor) hardwareElement.device;

    }

     /** Gets a hardware element.
      * @param name The name of the element.
      */
    public <T> T get(String name) {
        HardwareDevice device = hardwareElements.get(name);

        //noinspection unchecked
        return (T) device;
    }

    /** Gets the associated hardware map. */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /** Gets the associated telemetry. */
    public Telemetry getTelemetry() {
        return telemetry;
    }
}