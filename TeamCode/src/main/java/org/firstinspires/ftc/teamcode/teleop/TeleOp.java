package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpMode")
public class TeleOp extends OpMode {
    private Hardware robot;
    double leftFPower;
    double rightFPower;
    double leftBPower;
    double rightBPower;
    double drive;
    double turn;
    double strafe;
    double intakePower = 0.5;
    private final double armStartPos = 0.0;
    private final double armTargetPos = 0.0;
    private final double armTolerance = 2.5;
    private final double viperSlideStartPos = 0.0;
    private final double viperSlideTargetPos = 0.0;
    private final double jointStartPos = 0.0;
    private final double jointTargetPos = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Do all init stuff.
        robot = new Hardware(hardwareMap, telemetry);

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "jointServo"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "geekoWheelCRServo"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "tubeCRServo"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightBack", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armMotor", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlideMotor", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlideMotor", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls:", "\n" +
                "        Left Stick y = Drive\n" +
                "        Left Stick x = Strafe\n" +
                "        Right Stick x = Turn\n" +
                "        Right Bumper = Joint +\n" +
                "        Right Trigger = Joint -\n" +
                "        Left Bumper = Automatic Scoring\n" +
                "        Left Trigger = VS, Arm and Claw reset (Auto)\n" +
                "        A = Activate Intake (Hold)\n" +
                "        X = Claw + (Manual)\n" +
                "        Y = Claw - (Manual)\n" +
                "        D Pad Up = VS Up (Manual)\n" +
                "        D Pad Down = VS Down (Manual)");
    }

    @Override
    public void loop() {
        /* Controls:
         * Left Stick y = Drive
         * Left Stick x = Strafe
         * Right Stick x = Turn
         * Right Bumper = Joint +
         * Right Trigger = Joint -
         * Left Bumper = Automatic Scoring
         * Left Trigger = VS, Arm and Claw reset (Auto)
         * A = Activate Intake (Hold)
         * X = Claw + (Manual)
         * Y = Claw - (Manual)
         * D Pad Up = VS Up (Manual)
         * D Pad Down = VS Down (Manual)
         */

        // Values for drive.
        drive = gamepad1.left_stick_y * 0.8;
        turn = -gamepad1.right_stick_x * 0.6;
        strafe = gamepad1.left_stick_x * 0.8;

        // Set power to values calculated above.
        robot.<DcMotor>get("leftFront").setPower(leftFPower);
        robot.<DcMotor>get("leftRear").setPower(leftBPower);
        robot.<DcMotor>get("rightFront").setPower(rightFPower);
        robot.<DcMotor>get("rightRear").setPower(rightBPower);

        telemetry.addData("leftFPower: ", leftFPower);
        telemetry.addData("leftBPower: ", leftBPower);
        telemetry.addData("rightFPower: ", rightFPower);
        telemetry.addData("rightBPower: ", rightBPower);

        // Calling scoring() via left bumper and resetting via left trigger
        if (gamepad1.left_bumper) {
            scoring();
        } else if (gamepad1.left_trigger > 0.5) {
            resetPos();
            telemetry.addData("Scoring Pos", "Reset");
        }

        // Move claw via x and y.
        if (gamepad1.x) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() + -0.125);
        } else if (gamepad1.y) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() + 0.125);
        }

        // Moving VS maually via dpad up and down.
        if (gamepad1.dpad_up) {
            robot.<DcMotor>get("leftViperSlideMotor").setPower(0.5);
            robot.<DcMotor>get("rightViperSlideMotor").setPower(0.5);
        } else if (gamepad1.dpad_down) {
            robot.<DcMotor>get("leftViperSlideMotor").setPower(-0.5);
            robot.<DcMotor>get("rightViperSlideMotor").setPower(-0.5);
        } else {
            robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
            robot.<DcMotor>get("rightViperSlideMotor").setPower(0);
        }

        // Controls jointServo using right bumper and right trigger.
        if (gamepad1.right_bumper) {
            robot.<CRServo>get("jointServo").setPower(0.15);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("jointServo").setPower(-0.15);
        } else {
            robot.<CRServo>get("jointServo").setPower(0);
        }

        // Activating Intake via gamepad a.
        if (gamepad1.a) {
            robot.<CRServo>get("tubeCRServo").setPower(intakePower);
            robot.<CRServo>get("geekoWheelCRServo").setPower(intakePower);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("tubeCRServo").setPower(0);
            robot.<CRServo>get("geekoWheelCRServo").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }
    }

    // Scoring method, all together like a bowl of chili.
    public void scoring() {
        telemetry.addData("Scoring Status", "Started");

        setViperSlidePos();
        setArmPos();
        setJointPos();

        telemetry.addData("Scoring Status", "Ended");
    }

    // Setting VS POS.
    public void setViperSlidePos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Setting VS Pos");

        // Setting ViperSlidePoses based on Targets.
        while (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() < viperSlideTargetPos || robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() < viperSlideTargetPos) {
            runViperSlidesToPosition(viperSlideTargetPos, 0.4);
            viperSlideTelemetryUpdate();
        }

        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    // Setting Arm Pos.
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

    //Setting Joint Pos.
    public void setJointPos() {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        robot.<Servo>get("jointServo").setPosition(jointTargetPos); // Setting JointPos based on Target.
        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    // Resetting Scoring POS.
    public void resetPos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

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

    // VS Telemetry Updates.
    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlideMotor").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlideMotor").getPower());
    }

    // VS run Code to Target.
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

    // Strafe Drive using sticks on Gamepad 1
    public void driving() {
        // Calculate drive power.
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower = strafe;
            rightBPower = -strafe;
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }
    }
}
