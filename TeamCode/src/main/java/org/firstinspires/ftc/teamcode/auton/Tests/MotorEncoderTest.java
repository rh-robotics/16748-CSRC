package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.*;

@Autonomous(name="Motor Encoder Test")

public class MotorEncoderTest extends OpMode {
    float wheelEncoderPPR = 537.7f; // PPR
    int wheelDiameter = 96; // mm
    double mmPerEncoderTick = (360/wheelEncoderPPR)/360*(wheelDiameter*Math.PI); // 0.56089435511 mm
    double distance;
    float motorRunPower = 0.4f;
    private Hardware robot;

    @Override
    public void init(){
      robot = new Hardware(hardwareMap, telemetry);
      robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
      robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:REVERSE"));
      robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
      robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear"));
    }

    @Override
    public void start() {
        distance = inchesToMM(12);

        setTargetPosition(robot.<DcMotor>get("leftFront"), distance);
        setTargetPosition(robot.<DcMotor>get("rightFront"), distance);
        setTargetPosition(robot.<DcMotor>get("leftRear"), distance);
        setTargetPosition(robot.<DcMotor>get("rightRear"), distance);

        while (wheelsInPosition()) {
            updateMotors();
        }
    }

    public double inchesToMM(double inches) {
        return inches*25.4;
    }

    public void loop(){}
    public void setTargetPosition(DcMotor motor, double mm) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition((int) Math.round(mm/mmPerEncoderTick));
    }

    public boolean wheelsInPosition() {
        return robot.<DcMotor>get("leftFront").getTargetPosition() !=
                robot.<DcMotor>get("leftFront").getCurrentPosition() &&
                robot.<DcMotor>get("rightFront").getTargetPosition() !=
                        robot.<DcMotor>get("rightFront").getCurrentPosition() &&
                robot.<DcMotor>get("leftRear").getTargetPosition() !=
                        robot.<DcMotor>get("leftRear").getCurrentPosition() &&
                robot.<DcMotor>get("rightRear").getTargetPosition() !=
                        robot.<DcMotor>get("rightRear").getCurrentPosition();
    }

    public void updateMotors() {
        showMotorStatus(robot.<DcMotor>get("leftFront"));
        showMotorStatus(robot.<DcMotor>get("rightFront"));
        showMotorStatus(robot.<DcMotor>get("leftRear"));
        showMotorStatus(robot.<DcMotor>get("rightRear"));

        updateMotorPower(robot.<DcMotor>get("leftFront"));
        updateMotorPower(robot.<DcMotor>get("rightFront"));
        updateMotorPower(robot.<DcMotor>get("leftRear"));
        updateMotorPower(robot.<DcMotor>get("rightRear"));
    }

    public void showMotorStatus(DcMotor motor) {
        if (motor.getTargetPosition() == motor.getCurrentPosition()) {
            telemetry.addLine(motor.getDeviceName() + " in Position");
        } else {
            telemetry.addLine("Running " + motor.getDeviceName() + " Motor");
            telemetry.addData(motor.getDeviceName() + " Target Position",
                    motor.getTargetPosition());
            telemetry.addData(motor.getDeviceName() + " Current Position",
                    motor.getTargetPosition());
        }
    }

    public void updateMotorPower(DcMotor motor) {
        if (motor.getTargetPosition() == motor.getCurrentPosition()) {
            motor.setPower(0);
        } else {
            motor.setPower(motorRunPower);
        }
    }
}