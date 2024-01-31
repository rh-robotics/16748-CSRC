package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "DcMotor Test")
public class DcMotorTest extends OpMode {
    Hardware robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "testMotor"));
        robot.<DcMotor>get("testMotor").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        robot.<DcMotor>get("testMotor").setTargetPosition(-300);
        robot.<DcMotor>get("testMotor").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("testMotor").setPower(-0.75);
    }

    @Override
    public void loop() {
        if (robot.<DcMotor>get("testMotor").getCurrentPosition() > 100) {
            robot.<DcMotor>get("testMotor").setPower(0.75);
        } else if (robot.<DcMotor>get("testMotor").getCurrentPosition() <=
                robot.<DcMotor>get("testMotor").getTargetPosition()) {
            robot.<DcMotor>get("testMotor").setPower(0);
        }
        
        telemetry.addData("target pos", robot.<DcMotor>get("testMotor").getTargetPosition());
        telemetry.addData("current pos", robot.<DcMotor>get("testMotor").getCurrentPosition());
    }
}
