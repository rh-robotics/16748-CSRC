package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Color Sensor Test")
public class ColorSensorTest extends OpMode {
    ColorSensor colorSensor;
    int colorReading;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("color");
    }

    @Override
    public void loop() {
        //finds amount of red, green, blue, and light (alpha)
        colorReading = colorSensor.argb();
        telemetry.addData("Hue", colorReading);
        telemetry.update();
    }
}
