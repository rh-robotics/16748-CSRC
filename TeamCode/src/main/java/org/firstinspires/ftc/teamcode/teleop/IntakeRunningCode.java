package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Defines Class IntakeRunningCode. */
public class IntakeRunningCode extends OpMode{
    /** Defines a Motor called intakeMotor. */
    DcMotor intakeMotor;

    /** Adds intakeMotor to the HardwareMap. */
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    @Override
    public void loop() {
        while (gamepad1.a) {
            intakeMotor.setPower(0.5);
            telemetry.addData("Intake Motor", "Running");
        }
    }
}
