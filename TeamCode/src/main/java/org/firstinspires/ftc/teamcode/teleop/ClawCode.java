package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
@TeleOp(name = "Claw Code")
public class ClawCode extends OpMode {

    private Hardware robot;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "jointServo"));
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        /* Rotating servos based on user input. */
        if (gamepad1.a) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() + -0.25);
        } else if (gamepad1.b) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() + 0.25);
        }

        if (gamepad1.right_bumper) {
            robot.<CRServo>get("jointServo").setPower(0.15);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("jointServo").setPower(-0.15);
        } else {
            robot.<CRServo>get("jointServo").setPower(0);
        }

        telemetry.addData("clawServo position: ", robot.<Servo>get("clawServo").getPosition());
        telemetry.addData("jointServo position: ", robot.<CRServo>get("jointServo").getPower());
    }
}
