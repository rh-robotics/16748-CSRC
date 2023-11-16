package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTest extends OpMode {
    ColorSensor colorSensor;
    int colorReading;

    public void init() {
        colorSensor = hardwareMap.colorSensor.get("color");
    }

    public void loop() {
        //finds amount of red, green, blue, and light (alpha)
        colorReading = colorSensor.argb();
        telemetry.addData("Hue", colorReading);
        telemetry.update();
    }
}
