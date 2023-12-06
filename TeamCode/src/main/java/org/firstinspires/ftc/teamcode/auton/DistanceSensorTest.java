package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Distance Sensor Test")
public class DistanceSensorTest extends OpMode {
    DistanceSensor distanceSensor;
    @Override
    public void init() {
        telemetry.addData("Initializing Distance Sensor.", null);
        telemetry.addData("Distance Sensor Initialized.", null);
    }

    @Override
    public void loop() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance (in cm):", distance);
    }
}
