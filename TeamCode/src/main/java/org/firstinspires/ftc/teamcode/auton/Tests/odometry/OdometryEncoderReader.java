package org.firstinspires.ftc.teamcode.auton.Tests.odometry;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Odometry Encoder Reader")
public class OdometryEncoderReader extends LinearOpMode {
    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "rightRear");
        rightEncoder = new MotorEx(hardwareMap, "leftFront");
        perpEncoder = new MotorEx(hardwareMap, "leftRear");

        telemetry.addData("leftEncoder", leftEncoder.encoder.getDistance());
        telemetry.addData("rightEncoder", leftEncoder.encoder.getDistance());
        telemetry.addData("centerEncoder", leftEncoder.encoder.getDistance());
    }

}