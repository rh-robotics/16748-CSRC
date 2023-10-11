package org.firstinspires.ftc.teamcode.subsystems.hardware;

public class HardwareValueException extends RuntimeException {
    public HardwareValueException(String key, String value) {
        super("Unimplemented hardware element value method '" + key + ":" + value + "' encountered.");
    }
}