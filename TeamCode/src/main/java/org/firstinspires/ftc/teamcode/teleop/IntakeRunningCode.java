package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
@TeleOp(name = "Intake Running Code", group = "Iterative OpMode")
public class IntakeRunningCode extends OpMode{
    private final ElapsedTime time = new ElapsedTime();
    private Hardware robot;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        /** Introduced Servos. */
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "geckoWheelCRServo"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "tubeCRServo"));
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.<CRServo>get("tubeCRServo").setPower(0.5);
            robot.<CRServo>get("geckoWheelCRServo").setPower(0.5);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("tubeCRServo").setPower(0);
            robot.<CRServo>get("geckoWheelCRServo").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }
    }
}