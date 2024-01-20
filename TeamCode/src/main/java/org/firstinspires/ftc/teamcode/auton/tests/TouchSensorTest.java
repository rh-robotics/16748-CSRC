package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name="Touch Sensor Test")

public class TouchSensorTest extends OpMode {
    private Hardware robot;
    boolean pressed;

    @Override
    public void init() {
        telemetry.addData("Touch Sensor", "Initializing");
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(TouchSensor.class, hardwareMap, "touchSensor"));
        telemetry.addData("Touch Sensor", "Initialized");
    }

    @Override
    public void loop() {
        pressed = robot.<TouchSensor>get("touchSensor").isPressed();
        telemetry.addData("Touch Sensor Pressed", pressed);
    }
}