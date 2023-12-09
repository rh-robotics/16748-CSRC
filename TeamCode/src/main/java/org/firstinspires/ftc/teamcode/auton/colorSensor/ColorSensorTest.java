package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "Color Sensor Test")

public class ColorSensorTest extends OpMode {
    private Hardware robot;
    int red;
    int green;
    int blue;

    @Override
    public void init() {
        telemetry.addData("Color Sensor", "Initializing");
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(ColorSensor.class, hardwareMap, "colorSensor"));
        telemetry.addData("Color Sensor", "Initialized");
    }

    @Override
    public void loop() {
        // Finds amount of red, green, blue, and light (alpha).
        red = robot.<ColorSensor>get("colorSensor").red();
        green = robot.<ColorSensor>get("colorSensor").green();
        blue = robot.<ColorSensor>get("colorSensor").blue();
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
    }
}
