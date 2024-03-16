package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Auto")
public class AutoDemo extends LinearOpMode {
    public static double DISTANCE = 12; // in
    public int index = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34,24, Math.toRadians(180)))
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(trajectory.end())
                .lineTo(new Vector2d(24,24))
                .build();

        drive.setPoseEstimate(new Pose2d(10, 60, Math.toRadians(90)));
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

//    public Trajectory getTrajectory(SampleMecanumDrive drive, int index) {
//        ArrayList<Trajectory> trajectories = new ArrayList<>();
//
//        Trajectory blueFrontToMidLine = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(34,24, Math.toRadians(180)))
//                .splineTo(new Vector2d(24,24), Math.toRadians(180))
//                .build();
//
//        Trajectory blueFrontToLeftLine = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
//                .lineTo(new Vector2d(10,40))
//                .splineToLinearHeading(new Pose2d(10,35), 0)
//                .build();
//
//        Trajectory blueFrontToRightLine = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
//                .lineTo(new Vector2d(10,40))
//                .splineToLinearHeading(new Pose2d(10,35), 0)
//                .build();
//
//        Trajectory trajectory = null;
//
//        switch (index) {
//            case 0:
//                trajectory = blueFrontToMidLine;
//            case 1:
//                trajectory = blueFrontToLeftLine;
//            case 2:
//                trajectory = blueFrontToRightLine;
//        }
//
//        return trajectory;
//    }
}
