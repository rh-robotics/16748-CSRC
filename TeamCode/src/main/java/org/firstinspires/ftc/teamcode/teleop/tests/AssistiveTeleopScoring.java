package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@TeleOp(name="Assistive Scoring Test")
public class AssistiveTeleopScoring extends OpMode {
    private Hardware robot;
    private double targetPos;
    private double CPR = 1993.6;

    private final double armIntakePos = 0; // In revolutions
    private final double armScoringPos = -0.35;
    private final double VSIntakePos = 0;
    private final double VSScoringPos = 0.5;

    @Override
    public void init() {
        telemetry.addLine("Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setDirection:REVERSE"));


        telemetry.addLine("Initialized");
    }

    /** Reset motors to initial position here before we run anything else.
     * This step is necessary because we are using relative encoders, which give encoder information
     * based on where the motor is started/reset.
     **/
    @Override
    public void start() {
        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("armJoint").setTargetPosition(0);

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(0);

        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            scoringPosition();
        } else if (gamepad1.a) {
            intakePosition();
        }

        printTelemetry();

        if (gamepad1.a && gamepad1.b) {
            stopOpMode();
        }
    }

    public void printTelemetry() {
        telemetry.addData("Target Position", robot.<DcMotor>get("armJoint").getTargetPosition());
        telemetry.addData("Current Position", robot.<DcMotor>get("armJoint").getCurrentPosition());
        telemetry.addData("Arm Motor Power", robot.<DcMotor>get("armJoint").getPower());

        telemetry.addLine();
        telemetry.addData("VS Left Target Position", robot.<DcMotor>get("leftViperSlide").getTargetPosition());
        telemetry.addData("VS Left Current Position", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addLine();
        telemetry.addData("VS Right Target Position", robot.<DcMotor>get("rightViperSlide").getTargetPosition());
        telemetry.addData("VS Right Current Position", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    public void scoringPosition() {
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(revolutionsToTicks(VSScoringPos));
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(revolutionsToTicks(VSScoringPos));

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getTargetPosition() - robot.<DcMotor>get("leftViperSlide").getCurrentPosition())
                > Math.abs(VSScoringPos - VSIntakePos) / 2) {}

        robot.<DcMotor>get("armJoint").setTargetPosition(revolutionsToTicks(armScoringPos));

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);
    }

    public void intakePosition() {
        robot.<DcMotor>get("armJoint").setTargetPosition(revolutionsToTicks(armIntakePos));

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);


        robot.<DcMotor>get("leftViperSlide").setTargetPosition(revolutionsToTicks(VSIntakePos));
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(revolutionsToTicks(VSIntakePos));

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);
    }

    /** Reset motors to initial position here before ending the program. This helps for testing
     * when we want to run auton multiple times in a row, helps with redundancy for making sure
     * our encoders are accurate, and also is just convenient to have reset at the end of auton period. **/
    public void stopOpMode() {
        intakePosition();

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getCurrentPosition() - robot.<DcMotor>get("leftViperSlide").getTargetPosition()) < 5
        && Math.abs(robot.<DcMotor>get("armJoint").getCurrentPosition() - robot.<DcMotor>get("armJoint").getTargetPosition()) < 5) {
            printTelemetry();
        }

        requestOpModeStop();
    }

    public int revolutionsToTicks(double revolutions) {
        return (int) Math.round(revolutions * CPR);
    }
}
