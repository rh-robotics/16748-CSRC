package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
@TeleOp(name = "Claw Code", group = "Iterative OpMode")
public class ClawCode extends OpMode {

    private Hardware robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        /* Rotating servos based on user input. */
        if (gamepad1.a) {
            robot.<Servo>get("clawServo").setPosition(-0.25);
        } else if (gamepad1.b) {
            robot.<Servo>get("clawServo").setPosition(0.25);
        }
    }
}
