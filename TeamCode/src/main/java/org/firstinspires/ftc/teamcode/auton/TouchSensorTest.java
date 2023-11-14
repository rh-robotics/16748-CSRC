package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Touch Sensor Test")
public class TouchSensorTest extends OpMode {
    TouchSensor touch;
    @Override
    public void init() {}

    @Override
    public void loop() {
        touch = hardwareMap.touchSensor.get("touchSensor");
        telemetry.addData("Touch Sensor", touch.isPressed());
    }
}