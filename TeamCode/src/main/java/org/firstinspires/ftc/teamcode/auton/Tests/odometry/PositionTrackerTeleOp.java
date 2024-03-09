package org.firstinspires.ftc.teamcode.auton.Tests.odometry;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Position Tracker TeleOp")
public class PositionTrackerTeleOp extends LinearOpMode {

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

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        odometry.updatePose(PositionTracker.robotPose);

        telemetry.addData("Robot Position at Init: ", PositionTracker.robotPose);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // teleop things

            // update position
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();
        }
    }

}