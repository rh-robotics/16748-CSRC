package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class colorSensorTest extends LinearOpMode {
    ColorSensor color_sensor;

    public void runOpMode() {
        color_sensor = hardwareMap.colorSensor.get("color");

        waitForStart();

        while (opModeIsActive()) {
            //finds amount of red, green, blue, and light (alpha)
            telemetry.addData("red: ", color_sensor.red());
            telemetry.addData("green: ", color_sensor.green());
            telemetry.addData("blue: ", color_sensor.blue());
            telemetry.addData("alpha: ", color_sensor.alpha());
            telemetry.update();
        }
    }
}
