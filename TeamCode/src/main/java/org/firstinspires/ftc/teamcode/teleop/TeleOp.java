package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.PID;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpMode")
public class TeleOp extends OpMode {
    public ElapsedTime armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Hardware robot;

    private RobotMethods rMethods = new RobotMethods();

    // Drive Variables
    int targetRPM = 200;

    // Intake Variables
    double intakePower = 0.5;
    boolean intakeToggle = false;

    // Scoring Algorithm Variables
    private final double armStartPos = 0.0;
    private final double armTargetPos = 5.0;
    private final double armTolerance = 2.5;
    private final double viperSlideStartPos = 0.0;
    private final double viperSlideTargetPos = 0.0;
    private final double jointStartPos = 0.0;
    private final double jointTargetPos = 0.0;
    boolean scoringATM;
    byte lockPositionIndex = 0;
    byte slidePositionIndex = 0;
    byte armPositionIndex = 0;
    double P_Constant = 0.1;
    double I_Constant = 0.1;
    double D_Constant = 0.1;

    double[] lockPositions = new double[] {0.0, 0.4, 0.8};
    double[] slidePositions = new double[] {0.0, 1.0};
    double[] armPositions = new double[] {0.0, 1.0};

    ElapsedTime clawPositionChange = new ElapsedTime();
    ElapsedTime viperSlidePositionChange = new ElapsedTime();
    ElapsedTime armPositionChange = new ElapsedTime();

    // Gamepads
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousActionGamepad1  = new Gamepad();
    Gamepad previousActionGamepad2  = new Gamepad();

    boolean modeToggle = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        PIDController leftFrontPID = new PIDController(P_Constant, I_Constant, D_Constant);
        PIDController leftRearPID = new PIDController(P_Constant, I_Constant, D_Constant);
        PIDController rightFrontPID = new PIDController(P_Constant, I_Constant, D_Constant);
        PIDController rightRearPID = new PIDController(P_Constant, I_Constant, D_Constant);

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawLock"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "clawJoint"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:REVERSE, setMode:RUN_WITHOUT_ENCODER"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint", "setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide", "setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setMode:RUN_WITHOUT_ENCODER"));

        telemetry.addData("Status", "Initialized");

        listControls();
    }

    public void start() {
        armTime.reset();
        clawPositionChange.reset();
        viperSlidePositionChange.reset();
        armPositionChange.reset();
    }

    @Override
    public void loop() {
        if (currentGamepad1.back && !previousActionGamepad1.back) {
            modeToggle = !modeToggle;
        }

        if (!modeToggle) {
            telemetry.addData("Mode", modeToggle);
            listControls();
            gamepadUpdate();

            // automaticScoring();
            clawLock();
            intake();

            outerIntakeJoints();
            driving();
        } else {
            telemetry.addData("Mode", modeToggle);
            listControls();
            gamepadUpdate();

            setArmPos();
            clawLock();
            viperSlides();
            clawJoint();
            driving();
        }

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
        if (gamepad1.dpad_up) {
            robot.<DcMotor>get("leftViperSlide").setPower(0.25);
            robot.<DcMotor>get("rightViperSlide").setPower(-0.25);
        } else if (gamepad1.dpad_down) {
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
        if (currentGamepad1.a && !previousActionGamepad1.a) {
            intakeToggle = !intakeToggle;
        }

        if (intakeToggle) {
            robot.<CRServo>get("intakeTube").setPower(intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0.2);
            robot.<CRServo>get("outerIntakeTube").setPower(0.5);
            telemetry.addData("Intake", "Running");
        } else if (gamepad1.x) {
            robot.<CRServo>get("intakeTube").setPower(-intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(-0.2);
            robot.<CRServo>get("outerIntakeTube").setPower(-0.5);
            telemetry.addData("Intake", "Running");
        }
        else {
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("outerIntakeTube").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }

        if (gamepad1.y) {
            robot.<CRServo>get("intakeGeckoWheels").setPower(-0.2);
        }
    }

    public void outerIntakeJoints() {
        if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("outerIntakeJoint").setPower(-0.25);
        } else if (gamepad1.right_bumper) {
            robot.<CRServo>get("outerIntakeJoint").setPower(0.25);
        }
    }


    // Strafe Drive using sticks on Gamepad 1.
    public void driving() {
        double drive = -gamepad1.left_stick_y * 0.8;
        double turn = -gamepad1.left_stick_x * 0.6;
        double strafe = gamepad1.right_stick_x * 0.8;

        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;

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

        // Set power to values calculated above.
        robot.<DcMotor>get("leftFront").setPower(leftFPower);
        robot.<DcMotor>get("leftRear").setPower(leftBPower);
        robot.<DcMotor>get("rightFront").setPower(rightFPower);
        robot.<DcMotor>get("rightRear").setPower(rightBPower);

        telemetry.addData("leftFPower", leftFPower);
        telemetry.addData("leftBPower", leftBPower);
        telemetry.addData("rightFPower", rightFPower);
        telemetry.addData("rightBPower", rightBPower);
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
//        assert viperSlideTargetPos >= viperSlideStartPos;
//
//        telemetry.addData("Scoring Status", "Setting VS Pos");
//
//        // Setting ViperSlidePoses based on Targets.
//        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() < viperSlideTargetPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() < viperSlideTargetPos) {
//            runViperSlidesToPosition(viperSlideTargetPos, 0.4);
//            viperSlideTelemetryUpdate();
//        }
//
//        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    // Setting Arm Pos.
    public void setArmPos() {
//        boolean armToggle = currentGamepad1.dpad_left;
//
//        telemetry.addData("armToggle", armToggle);
//        telemetry.addData("targetPos", robot.<DcMotor>get("armJoint").getTargetPosition());
//        telemetry.addData("armToggle", robot.<DcMotor>get("armJoint").getCurrentPosition());
//        if (armToggle && rMethods.motorCloseEnoughPosition(robot.<DcMotor>get("armJoint"), armTolerance, armStartPos)) {
//            setTargetPos(robot.<DcMotor>get("armJoint"), (int) Math.round(armTargetPos));
//
//            robot.<DcMotor>get("armJoint").setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            robot.<DcMotor>get("armJoint").setPower(0.5);
//        } else if (!armToggle && rMethods.motorCloseEnoughPosition(robot.<DcMotor>get("armJoint"), armTolerance, armTargetPos)) {
//            robot.<DcMotor>get("armJoint").setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            robot.<DcMotor>get("armJoint").setPower(-0.5);
//        }
//
//        if (armTime.time() > 0.35) {
//            // Moving arm manually.
//            if (currentGamepad1.dpad_left) {
//                armTime.reset();
//                while (armTime.time() < 0.3) {
//                    robot.<DcMotor>get("armJoint").setPower(0.7);
//                }
//            } else if (currentGamepad1.dpad_right) {
//                armTime.reset();
//                while (armTime.time() < 0.3) {
//                    robot.<DcMotor>get("armJoint").setPower(-0.7);
//                }
//            }
//        }
//        robot.<DcMotor>get("armJoint").setPower(0);
    }

    public void setTargetPos(DcMotor motor, int target){
        motor.setTargetPosition(target);
    }

    public void setPIDPower(DcMotor motor, int direction) {
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double target = motor.getTargetPosition();

        PIDController controller = new PIDController(0.1, 0, 0);
        int Pos = motor.getCurrentPosition();

        motor.setPower(controller.calculate(Pos, target) * direction);
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
        if (!modeToggle) {
            telemetry.addLine("Controls:\n" +
                    "         *** \n" +
                    "         * Gamepad 1\n" +
                    "         * Left Stick y = Drive\n" +
                    "         * Left Stick x = Strafe\n" +
                    "         * Right Stick x = Turn\n" +
                    "         * Left Trigger = Claw Lock -\n" +
                    "         * Left Bumper = Claw Lock +\n" +
                    "         * Right Trigger = Outer Intake -\n" +
                    "         * Right Bumper = Outer Intake +\n" +
                    "         * A = Run Intake (Toggle)\n" +
                    "         * B = Auto Score Toggle\n" +
                    "         * X = Intake Back Flush\n" +
                    "         * Y = --\n" +
                    "         * D Pad Left = ---\n" +
                    "         * D Pad Right = ---\n" +
                    "         * D Pad Up = Claw Joint +\n" +
                    "         * D Pad Down = Claw Joint -");
        } else {
            telemetry.addLine("Controls:\n" +
                    "         *** \n" +
                    "         * Gamepad 2\n" +
                    "         * Left Stick y = Drive\n" +
                    "         * Left Stick x = Strafe\n" +
                    "         * Right Stick x = Turn\n" +
                    "         * Left Trigger = Claw Lock -\n" +
                    "         * Left Bumper = Claw Lock +\n" +
                    "         * Right Trigger = Claw Joint -\n" +
                    "         * Right Bumper = Claw Joint +\n" +
                    "         * A = Arm Toggle\n" +
                    "         * B = VS Toggle\n" +
                    "         * D Pad Left = Arm - \n" +
                    "         * D Pad Right = Arm + \n" +
                    "         * D Pad Up = VS + \n" +
                    "         * D Pad Down = VS -");
        }
    }

    public void gamepadUpdate() {
        previousActionGamepad1.copy(currentGamepad1);
        previousActionGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }
}