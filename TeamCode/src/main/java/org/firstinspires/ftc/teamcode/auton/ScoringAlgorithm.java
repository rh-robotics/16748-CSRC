package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "Scoring Algorithm", group = "auton")
public class ScoringAlgorithm extends OpMode {
    private Hardware robot;
    // TODO: Get values for variables.
    private boolean pixelsInClaw = false;
    private final double armStartPos = 0.0;
    private final double armTargetPos = 0.0;
    private final double viperSlideStartPos = 0.0;
    private final double viperSlideTargetPos = 0.0;
    private final double jointStartPos = 0.0;
    private final double jointTargetPos = 0.0;

    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        // Introducing servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "jointServo"));

        // Introducing motors and encoders.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armMotor"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide"));

        // Motor and encoder initialization.
        robot.<DcMotor>get("armMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("armMotor").setDirection(DcMotor.Direction.FORWARD);
        robot.<DcMotor>get("leftViperSlide").setDirection(DcMotorSimple.Direction.FORWARD);
        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("rightViperSlide").setDirection(DcMotorSimple.Direction.FORWARD);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            scoring();
        }
    }

    public void scoring() {
        telemetry.addData("Scoring Status", "Started");

        setViperSlidePos();
        setArmPos();
        setJointPos();

        while (pixelsInClaw) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() - 0.125);
        }

        resetPos();
        telemetry.addData("Scoring Status", "Ended");
    }

    public void setViperSlidePos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Setting VS Pos");

        // Setting ViperSlidePoses based on Targets.
        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() < viperSlideTargetPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() < viperSlideTargetPos) {
            runViperSlidesToPosition(viperSlideTargetPos, 0.4);
            viperSlideTelemetryUpdate();
        }

        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    public void setArmPos() {
        // If this is not true, the rest of the code will not function.
        assert armTargetPos >= armStartPos;

        telemetry.addData("Scoring Status", "Setting Arm Pos");

        // Setting ArmPos based on Target.
        while (robot.<DcMotor>get("armMotor").getCurrentPosition() < armTargetPos) {
            robot.<DcMotor>get("armMotor").setPower(0.6);
            telemetry.addData("armMotor Pos", robot.<DcMotor>get("armMotor").getCurrentPosition());
        }

        robot.<DcMotor>get("armMotor").setPower(0);
        telemetry.addData("Scoring Status", "Arm Pos Set");
    }

    public void setJointPos() {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        robot.<Servo>get("jointServo").setPosition(jointTargetPos); // Setting JointPos based on Target.
        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    public void resetPos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Resetting Pos");
        robot.<Servo>get("jointServo").setPosition(jointStartPos);

        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > viperSlideStartPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > viperSlideStartPos) {
            runViperSlidesToPosition(viperSlideStartPos, -0.4);
            viperSlideTelemetryUpdate();
        }

        robot.<DcMotor>get("leftViperSlide").setPower(0);
        robot.<DcMotor>get("rightViperSlide").setPower(0);
        telemetry.addData("Scoring Status", "Pos Reset");
    }

    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    public void runViperSlidesToPosition(double position, double power) {
        robot.<DcMotor>get("leftViperSlide").setPower(0);
        robot.<DcMotor>get("rightViperSlide").setPower(0);

        if (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > position) {
            robot.<DcMotor>get("leftViperSlide").setPower(power);
            telemetry.addLine("Running VS Left");
        }

        if (robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > position) {
            robot.<DcMotor>get("rightViperSlide").setPower(power);
        }
    }
}