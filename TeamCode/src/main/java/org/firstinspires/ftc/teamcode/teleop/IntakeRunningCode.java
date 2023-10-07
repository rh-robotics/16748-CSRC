package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Defines Class IntakeRunningCode

public class IntakeRunningCode extends OpMode{

    // Defines a Moter called intakeMotor

    DcMotor intakeMotor;

    // Adds intakeMotor to the HardwareMap

    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }
    @Override
    public void loop(){

        /* While the button a is pressed the motor will run a 0.5 and
        will add telemetry that the intake motor is running */

        while (gamepad1.a) {
            intakeMotor.setPower(0.5);
            telemetry.addData("Intake Motor", "Running");
        }
    }
}
