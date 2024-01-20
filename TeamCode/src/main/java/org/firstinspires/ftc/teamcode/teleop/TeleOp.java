package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

// TODO: Outer intake

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
    boolean scoringATM;
    byte lockPositionIndex = 0;
    byte slidePositionIndex = 0;
    byte armPositionIndex = 0;

    double[] lockPositions = new double[] {0.0, 0.4, 0.8};
    double[] slidePositions = new double[] {0.0, 1.0};
    double[] armPositions = new double[] {0.0, 1.0};

    ElapsedTime clawPositionChange = new ElapsedTime();
    ElapsedTime viperSlidePositionChange = new ElapsedTime();
    ElapsedTime armPositionChange = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawLock"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "clawJoint"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setMode:RUN_USING_ENCODER"));

        telemetry.addData("Status", "Initialized");

        listControls();
    }

    public void start() {
        clawPositionChange.reset();
        viperSlidePositionChange.reset();
        armPositionChange.reset();
    }

    @Override
    public void loop() {
        listControls();

        // automaticScoring();
        clawLock();
        viperSlides();
        clawJoint();
        intake();
        arm();
        outerIntakeJoints();
        driving();
    }

    public void automaticScoring() {
        // Calling scoring() via left bumper and resetting via left trigger
        if (gamepad1.dpad_left) {
            scoring();
        } else if (gamepad1.y && !scoringATM) {
            resetPos();
            telemetry.addData("Scoring Pos", "Reset");
        }
    }

    public void clawLock() {
        // Move claw via x and y.
        if (clawPositionChange.time() > 0.25){
            if (gamepad1.left_bumper) {
                lockPositionIndex ++;
                if(lockPositionIndex > 2) {
                    lockPositionIndex = 2;
                }
                robot.<Servo>get("clawLock").setPosition(lockPositions[lockPositionIndex]);
                clawPositionChange.reset();
            } else if (gamepad1.left_trigger > 0.5) {
                lockPositionIndex --;
                if(lockPositionIndex < 0) {
                    lockPositionIndex = 0;
                }
                robot.<Servo>get("clawLock").setPosition(lockPositions[lockPositionIndex]);
                clawPositionChange.reset();
            }
        }
        telemetry.addData("claw lock position", robot.<Servo>get("clawLock").getPosition());
        telemetry.addData("claw lock index", lockPositionIndex);
    }

    public void viperSlides() {
        // Moving VS to pre-set positions.
        if (viperSlidePositionChange.time() > 0.5){
            if (false) { // gamepad1.b
                if (lockPositionIndex == 1) {
                    lockPositionIndex = 0;
                } else {
                    lockPositionIndex = 1;
                }

                robot.<DcMotor>get("leftViperSlide").setTargetPosition((int) slidePositions[slidePositionIndex]);
                robot.<DcMotor>get("rightViperSlide").setTargetPosition((int) slidePositions[slidePositionIndex]);
                viperSlidePositionChange.reset();
            }
        }

        // Moving VS manually via dpad up and down.
        if (gamepad1.b) {
            robot.<DcMotor>get("leftViperSlide").setPower(0.25);
            robot.<DcMotor>get("rightViperSlide").setPower(-0.25);
        } else if (gamepad1.y) {
            robot.<DcMotor>get("leftViperSlide").setPower(-0.25);
            robot.<DcMotor>get("rightViperSlide").setPower(0.25);
        } else {
            robot.<DcMotor>get("leftViperSlide").setPower(0);
            robot.<DcMotor>get("rightViperSlide").setPower(0);
        }
    }

    public void clawJoint() {
        // Controls clawJoint using right bumper and right trigger.
        if (gamepad1.right_bumper) {
            robot.<CRServo>get("clawJoint").setPower(0.15);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("clawJoint").setPower(-0.15);
        } else {
            robot.<CRServo>get("clawJoint").setPower(0);
        }
    }

    public void intake() {
        // Activating Intake via gamepad a.
        if (gamepad1.a) {
            robot.<CRServo>get("intakeTube").setPower(intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0.2);
            robot.<CRServo>get("outerIntakeTube").setPower(0.5);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("outerIntakeTube").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }

        if (gamepad1.y) {
            robot.<CRServo>get("intakeGeckoWheels").setPower(-0.2);
        }
    }

    public void arm() {
        // Moving arm to pre-set positions.
        if (armPositionChange.time() > 0.5){
//            if (gamepad1.y) {
//                if (armPositionIndex == 1) {
//                    armPositionIndex = 0;
//                } else {
//                    armPositionIndex = 1;
//                }
//
//                robot.<DcMotor>get("armJoint").setTargetPosition((int) armPositions[armPositionIndex]);
//                armPositionChange.reset();
//            }
        }

        // Moving arm manually.
        if (gamepad1.dpad_left) {
            robot.<DcMotor>get("armJoint").setPower(0.15);
        } else if (gamepad1.dpad_right) {
            robot.<DcMotor>get("armJoint").setPower(-0.15);
        } else {
            robot.<DcMotor>get("armJoint").setPower(0);
        }
    }

    public void outerIntakeJoints() {
        if (gamepad1.dpad_up) {
            robot.<CRServo>get("outerIntakeJoint").setPower(-0.25);
        } else if (gamepad1.dpad_down) {
            robot.<CRServo>get("outerIntakeJoint").setPower(0.25);
        }
    }

    // Strafe Drive using sticks on Gamepad 1.
    public void driving() {
        // Values for drive.
        drive = gamepad1.left_stick_y * 0.8;
        turn = -gamepad1.right_stick_x * 0.6;
        strafe = gamepad1.left_stick_x * 0.8;

        // Set power to values calculated above.
        robot.<DcMotor>get("leftFront").setPower(leftFPower);
        robot.<DcMotor>get("leftRear").setPower(leftBPower);
        robot.<DcMotor>get("rightFront").setPower(rightFPower);
        robot.<DcMotor>get("rightRear").setPower(-rightBPower);

        telemetry.addData("leftFPower: ", leftFPower);
        telemetry.addData("leftBPower: ", leftBPower);
        telemetry.addData("rightFPower: ", rightFPower);
        telemetry.addData("rightBPower: ", rightBPower);

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

    // Scoring method, all together like a bowl of chili.
    public void scoring() {
        telemetry.addData("Scoring Status", "Started");

        scoringATM = true;

        setViperSlidePos();
        setArmPos();
        setJointPos();

        scoringATM = false;

        telemetry.addData("Scoring Status", "Ended");
    }

    // Setting VS POS.
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

    // Setting Arm Pos.
    public void setArmPos() {
        // If this is not true, the rest of the code will not function.
        assert armTargetPos >= armStartPos;

        telemetry.addData("Scoring Status", "Setting Arm Pos");

        // Setting ArmPos based on Target.
        while (robot.<DcMotor>get("armJoint").getCurrentPosition() < armTargetPos) {
            robot.<DcMotor>get("armJoint").setPower(0.6);
            telemetry.addData("armJoint Pos", robot.<DcMotor>get("armJoint").getCurrentPosition());
        }

        robot.<DcMotor>get("armJoint").setPower(0);
        telemetry.addData("Scoring Status", "Arm Pos Set");
    }

    // Setting Joint Pos.
    public void setJointPos() {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        robot.<Servo>get("clawJoint").setPosition(jointTargetPos); // Setting JointPos based on Target.
        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    // Resetting Scoring POS.
    public void resetPos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Resetting Pos");
        robot.<Servo>get("clawJoint").setPosition(jointStartPos);

        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > viperSlideStartPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > viperSlideStartPos) {
            runViperSlidesToPosition(viperSlideStartPos, -0.4);
            viperSlideTelemetryUpdate();
        }

        robot.<DcMotor>get("leftViperSlide").setPower(0);
        robot.<DcMotor>get("rightViperSlide").setPower(0);
        telemetry.addData("Scoring Status", "Pos Reset");
    }

    // VS Telemetry Updates.
    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    // VS run Code to Target.
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

    public void listControls() {
        telemetry.addLine("Controls:\n" +
                "         *** \n" +
                "         * Gamepad 1\n" +
                "         * Left Stick y = Drive\n" +
                "         * Left Stick x = Strafe\n" +
                "         * Right Stick x = Turn\n" +
                "         * Left Trigger = Claw Lock -\n" +
                "         * Left Bumper = Claw Lock +\n" +
                "         * Right Trigger = Claw Joint -\n" +
                "         * Right Bumper = Claw Joint +\n" +
                "         * A = Run Intake (Toggle)\n" +
                "         * B = VS +\n" +
                "         * X = VS -\n" +
                "         * Y = Arm Position Change\n" +
                "         * D Pad Left = Arm -\n" +
                "         * D Pad Right = Arm +\n" +
                "         * D Pad Up = Outer Intake Down\n" +
                "         * D Pad Down = Outer Intake Up");
    }
}
