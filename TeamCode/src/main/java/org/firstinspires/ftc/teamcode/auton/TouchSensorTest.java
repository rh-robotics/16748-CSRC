package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name="Touch Sensor Test")

public class TouchSensorTest extends OpMode {
    private Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(TouchSensor.class, hardwareMap, "touchSensor"));
        telemetry.addLine("Touch Sensor Initializing");
        telemetry.addLine("Touch Sensor Initialized");
    }

    @Override
    public void loop() {
        boolean touch = robot.<TouchSensor>get("touchSensor").isPressed();
        telemetry.addData("Touch Sensor Pressed", touch);
    }
}