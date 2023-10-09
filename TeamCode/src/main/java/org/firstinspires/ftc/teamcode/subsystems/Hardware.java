package org.firstinspires.ftc.teamcode.subsystems;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;

public class Hardware {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    /** ArrayList to hold all hardware elements. */
    private ArrayList<HardwareElement> hardwareElements = new ArrayList<HardwareElement>();

    public Hardware(@NonNull Telemetry telemetry, @NonNull HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /** Adds hardwareElement to hardwareElements arrayList */
    public void introduce(HardwareElement hardwareElement){
        this.hardwareElements.add(hardwareElement);
    }
}

/* eg. initialization code
 *
 * public Hardware robot = new Hardware(telemetry, hardwareMap);
 * robot.introduce(new HardwareElement("leftFront", "leftFront", DcMotorEx.class));
 * robot.introduce(new HardwareElement("rightFront", "rightFront", DcMotorEx.class,
 *          "setDirection:REVERSE"));
 *
 * robot.hardwareElements.leftFront.setPower(0.5);
 */