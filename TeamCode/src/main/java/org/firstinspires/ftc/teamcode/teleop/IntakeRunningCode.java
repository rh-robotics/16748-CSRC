package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

public class IntakeRunningCode extends OpMode{
    private final ElapsedTime time = new ElapsedTime();
    HWC robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        robot = new HWC(hardwareMap, telemetry);
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
    public void loop(){
        if (gamepad1.a) {
            robot.intakeMotor.setPower(0.5);
            telemetry.addData("Intake Motor", "Running");
        }
        else {
            robot.intakeMotor.setPower(0);
            telemetry.addData("Intake Motor", "At rest");
        }
    }
}
