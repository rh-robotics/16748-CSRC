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
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint1"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint2"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube1"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube2"));

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
        telemetry.addLine("Button A: Run Intake \n Right Bumper: Outer Intake Up \n " +
                "Right Trigger: Outer Intake Down");

        if (gamepad1.a) {
            robot.<CRServo>get("intakeTube").setPower(0.5);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0.5);
            robot.<CRServo>get("intakeTube").setPower(0.5);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0.5);
            robot.<CRServo>get("outerIntakeTube1").setPower(-0.5);
            robot.<CRServo>get("outerIntakeTube2").setPower(-0.5);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("outerIntakeTube1").setPower(0);
            robot.<CRServo>get("outerIntakeTube2").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }

        if (gamepad1.right_bumper) {
            robot.<CRServo>get("outerIntakeJoint1").setPower(-0.25);
            robot.<CRServo>get("outerIntakeJoint2").setPower(0.25);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("outerIntakeJoint1").setPower(0.25);
            robot.<CRServo>get("outerIntakeJoint2").setPower(-0.25);
        }
    }
}