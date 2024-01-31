package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "Prop Dectection")
public class PropDetection extends OpMode {

    private Hardware robot;
    int red;
    int blue;
    int colorTolerence = 30;
    boolean teamPropDetected = false;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(ColorSensor.class, hardwareMap, "colorSensor"));

        telemetry.addData("Status", "Init");
    }

    @Override
    public void loop() {
        red = robot.<ColorSensor>get("colorSensor").red();

        if (red - colorTolerence < red && red < red + colorTolerence) {
            teamPropDetected = true;
        }

        if (blue - colorTolerence < blue && blue < blue + colorTolerence) {
            teamPropDetected = true;
        }
    }
}
