package org.firstinspires.ftc.teamcode.subsystems;

public class HardwareValueException extends RuntimeException {
    public HardwareValueException(String key, String value) {
        super("Unimplemented hardware element method '" + key + ":" + value + "' encountered.");
    }
}