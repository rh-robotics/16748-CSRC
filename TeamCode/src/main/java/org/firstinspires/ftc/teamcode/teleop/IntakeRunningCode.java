package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.hardware.*;
@TeleOp(name = "Intake Running Code", group = "Iterative OpMode")
public class IntakeRunningCode extends OpMode{
    private final ElapsedTime time = new ElapsedTime();
    private Hardware robot;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        /** DcMotorEx is a child class of DcMotor, so for now we just introduce them as DcMotors.*/
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "intakeMotor"));

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
            robot.<DcMotor>get("intakeMotor").setPower(0.5);
            telemetry.addData("Intake Motor", "Running");
        } else {
            robot.<DcMotor>get("intakeMotor").setPower(0);
            telemetry.addData("Intake Motor", "At rest");
        }
    }
}