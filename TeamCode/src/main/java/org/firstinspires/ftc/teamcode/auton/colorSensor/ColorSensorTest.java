package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTest extends LinearOpMode {
    ColorSensor colorSensor;

    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("color");

        waitForStart();

        while (opModeIsActive()) {
            //finds amount of red, green, blue, and light (alpha)
            telemetry.addData("hue: ", colorSensor.argb());
            telemetry.update();
        }
    }
}
