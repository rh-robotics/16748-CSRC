package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "Touch Sensor Test", group = "Testing Touch Sensor")
public class TouchSensorTest extends OpMode {
    Hardware robot;

    @Override
    public void init() {
        telemetry.addLine("Initializing robot");
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(TouchSensor.class, hardwareMap, "touchSensor"));
        telemetry.addLine("Initialized robot");
    }

    @Override
    public void loop() {
        telemetry.addData("Touch sensor", robot.<TouchSensor>get("touchSensor").isPressed() ? "pressed" : "not-pressed");
    }
}
