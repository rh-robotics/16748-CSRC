package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;

import javax.xml.transform.sax.TransformerHandler;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "BlueFront Auto")
public class BlueFront extends LinearOpMode {
    public static double DISTANCE = 12; // in
    public int index = 0;

    public enum PROP_LOCATIONS {
        FAR,
        CENTER,
        CLOSE
    }

    PROP_LOCATIONS propLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMethods robotMethods = new RobotMethods();

        Trajectory midLine1 = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34,24, Math.toRadians(180)))
                .build();

        Trajectory midLine2 = drive.trajectoryBuilder(midLine1.end())
                .lineTo(new Vector2d(18,24))
                .build();

        Trajectory midToBackboard = drive.trajectoryBuilder(midLine2.end())
                .splineToLinearHeading(new Pose2d(43,35, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory farLine1 = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34,28, Math.toRadians(180)))
                .build();

        Trajectory farLine2 = drive.trajectoryBuilder(farLine1.end())
                .lineTo(new Vector2d(12,28))
                .build();

        Trajectory farToBackboard = drive.trajectoryBuilder(farLine2.end())
                .splineToLinearHeading(new Pose2d(40,35, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory closeLine1 = drive.trajectoryBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34,28, Math.toRadians(180)))
                .build();

        Trajectory closeLine2 = drive.trajectoryBuilder(closeLine1.end())
                .lineToLinearHeading(new Pose2d(43,35, Math.toRadians(180)))
                .build();

        Trajectory closeToBackboard = drive.trajectoryBuilder(closeLine2.end())
                .splineToLinearHeading(new Pose2d(40,35, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory backboardToPark1 = drive.trajectoryBuilder(midToBackboard.end())
                .lineTo(new Vector2d(43, 55))
                .build();

        Trajectory backboardToPark2 = drive.trajectoryBuilder(backboardToPark1.end())
                .lineTo(new Vector2d(53, 55))
                .build();

        drive.setPoseEstimate(new Pose2d(10, 60, Math.toRadians(90)));

        telemetry.addData("Status", 1);
        waitForStart();
        telemetry.addData("Status", 2);
        if (isStopRequested()) return;

        propLocation = PROP_LOCATIONS.CLOSE;

        telemetry.addData("Status", 3);
        robotMethods.outerIntakeDown(drive);
//        robotMethods.flushPixel(drive);
        robotMethods.scoringPosition(drive);
//        if (propLocation == PROP_LOCATIONS.CENTER) {
//            telemetry.addData("Status", 4);
//            drive.followTrajectory(midLine1);
//            drive.followTrajectory(midLine2);
//            robotMethods.flushPixel(drive);
//
//            drive.followTrajectory(midToBackboard);
//        } else if (propLocation == PROP_LOCATIONS.FAR) {
//            drive.followTrajectory(farLine1);
//            drive.followTrajectory(farLine2);
//            robotMethods.flushPixel(drive);
//
//            drive.followTrajectory(farToBackboard);
//        } else {
//            drive.followTrajectory(closeLine1);
//            drive.followTrajectory(closeLine2);
//            robotMethods.flushPixel(drive);
//
//            drive.followTrajectory(closeToBackboard);
//        }

//        drive.followTrajectory(backboardToPark1);
//        drive.followTrajectory(backboardToPark2);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());
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
