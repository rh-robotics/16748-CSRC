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
    boolean pixelsInClaw = false;
    double armStartPos = 0.0;
    double armTargetPos = 0.0;
    double armTolerance = 2.5;
    double viperSlideStartPos = 0.0;
    double viperSlideTargetPos = 0.0;
    double jointStartPos = 0.0;
    double jointTargetPos = 0.0;
    boolean scored = false;

    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        // Introducing servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "jointServo"));

        // Introducing motors and encoders.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armMotor"));
        robot.<DcMotor>get("armMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("armMotor").setDirection(DcMotor.Direction.FORWARD);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlideMotor"));
        robot.<DcMotor>get("rightViperSlideMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("leftViperSlideMotor").setDirection(DcMotorSimple.Direction.FORWARD);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlideMotor"));
        robot.<DcMotor>get("leftViperSlideMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("rightViperSlideMotor").setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

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
        scored = true;
    }

    public void setViperSlidePos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos > viperSlideStartPos;

        telemetry.addData("Scoring Status", "Setting VS Pos");
        // Setting ViperSlidePoses based on Targets.
        while (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() < viperSlideTargetPos || robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() < viperSlideTargetPos) {
            runViperSlidesToPosition(viperSlideTargetPos, 0.4);
            viperSlideTelemetryUpdate();
        }
        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    public void setArmPos() {
        // If this is not true, the rest of the code will not function.
        assert armTargetPos > armStartPos;

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

        // Setting JointPos based on Target.
        robot.<Servo>get("jointServo").setPosition(jointTargetPos);

        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    public void resetPos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos > viperSlideStartPos;

        telemetry.addData("Scoring Status", "Resetting Pos");
        robot.<Servo>get("jointServo").setPosition(jointStartPos);

        while (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() > viperSlideStartPos || robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() > viperSlideStartPos) {
            runViperSlidesToPosition(viperSlideStartPos, -0.4);
            viperSlideTelemetryUpdate();
        }

        robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
        robot.<DcMotor>get("rightViperSlideMotor").setPower(0);
        telemetry.addData("Scoring Status", "Pos Reset");
    }

    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlideMotor").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlideMotor").getPower());
    }

    public void runViperSlidesToPosition(double position, double power) {
        robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
        robot.<DcMotor>get("rightViperSlideMotor").setPower(0);

        if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() > position) {
            robot.<DcMotor>get("leftViperSlideMotor").setPower(power);
            telemetry.addLine("Running VS Left");
        }

        if (robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() > position) {
            robot.<DcMotor>get("rightViperSlideMotor").setPower(power);
        }
    }
}