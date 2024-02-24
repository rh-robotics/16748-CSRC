package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name="Distance Sensor Test")

public class DistanceSensorTest extends OpMode {
    private Hardware robot;

    @Override
    public void init() {
        telemetry.addData("Distance Sensor", "Initializing");
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(DistanceSensor.class, hardwareMap, "distanceSensor"));

        telemetry.addData("Distance Sensor", "Initialized");
    }

    @Override
    public void loop() {
        double distance = robot.<DistanceSensor>get("distanceSensor").getDistance(DistanceUnit.CM);
        telemetry.addData("Distance (in cm)", distance);
    }
}
