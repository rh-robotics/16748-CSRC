package org.firstinspires.ftc.teamcode.auton.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Prop Detection Test")
public class PropDetectionCameraTest extends OpMode {
    public OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
    String webcamName = "webcam1";

    propPlaces propPlacement;
    robotStartPlaces robotStartPosition;
    enum propPlaces {
    RIGHT,
    CENTER,
    LEFT
    }
    enum robotStartPlaces {
        RED_BACKDROP,
        RED_HUMANPLAYER,
        BLUE_BACKDROP,
        BLUE_HUMANPLAYER
    }

    private final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255);

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        webcam1.setPipeline(new ColorDetection());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            robotStartPosition = robotStartPlaces.RED_BACKDROP;
        } else if (gamepad1.dpad_down) {
            robotStartPosition = robotStartPlaces.RED_HUMANPLAYER;
        } else if (gamepad1.dpad_left) {
            robotStartPosition = robotStartPlaces.BLUE_BACKDROP;
        } else if (gamepad1.dpad_right) {
            robotStartPosition = robotStartPlaces.BLUE_HUMANPLAYER;
        }

        telemetry.addData("Robot Start Position", robotStartPosition);

        telemetry.addData("Position", propPlacement);
    }

    public class ColorDetection extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat YCbCr = new Mat();
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            // Define sectors left, center, and right sectors
            Rect leftRect = new Rect(1, 119, 104, 119);
            Rect centerRect = new Rect(105, 119, 109, 119);
            Rect rightRect = new Rect(215, 119, 104, 119);

            Mat leftCrop = YCbCr.submat(leftRect);
            Mat centerCrop = YCbCr.submat(centerRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            // Greyscale
//            Mat leftGray = new Mat();
//            Mat centerGray = new Mat();
//            Mat rightGray = new Mat();
//            Imgproc.cvtColor(leftCrop, leftGray, Imgproc.COLOR_BGR2GRAY);
//            Imgproc.cvtColor(centerCrop, centerGray, Imgproc.COLOR_BGR2GRAY);
//            Imgproc.cvtColor(rightCrop, rightGray, Imgproc.COLOR_BGR2GRAY);
//
//            // Isolate the red and blue
//            int redThreshold = 100;
//            int blueThreshold = 100;
//            Mat leftBinaryRed = new Mat();
//            Mat centerBinaryRed = new Mat();
//            Mat rightBinaryRed = new Mat();
//            Mat leftBinaryBlue = new Mat();
//            Mat centerBinaryBlue = new Mat();
//            Mat rightBinaryBlue = new Mat();
//            Imgproc.threshold(leftGray, leftBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(centerGray, centerBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(rightGray, rightBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(leftGray, leftBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(centerGray, centerBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(rightGray, rightBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);
//
//            // Gets color specific values
//            int redRightPercent = Core.countNonZero(rightBinaryRed);
//            int redCenterPercent = Core.countNonZero(centerBinaryRed);
//            int redLeftPercent = Core.countNonZero(leftBinaryRed);
//
//            int blueRightPercent = Core.countNonZero(rightBinaryBlue);
//            int blueCenterPercent = Core.countNonZero(centerBinaryBlue);
//            int blueLeftPercent = Core.countNonZero(leftBinaryBlue);
//
//            // Calculates the highest amount of pixels being covered on each side
//            // double maxPercentRed = Math.max(Math.max(redRightPercent, redCenterPercent), redLeftPercent);
//            // double maxPercentBlue = Math.max(Math.max(blueRightPercent, blueCenterPercent), blueLeftPercent);
//
//            telemetry.addData("Red Right Percent", redRightPercent);
//            telemetry.addData("Red Center Percent", redCenterPercent);
//            telemetry.addData("Red Left Percent", redLeftPercent);
//            telemetry.addData("Blue Right Percent", blueRightPercent);
//            telemetry.addData("Blue Center Percent", blueCenterPercent);
//            telemetry.addData("Blue Left Percent", blueLeftPercent);
//            telemetry.addData("Max Percent Red: ", maxPercentRed);
//            telemetry.addData("Max Percent Blue: ", maxPercentBlue);

            if (robotStartPosition == robotStartPlaces.RED_HUMANPLAYER || robotStartPosition == robotStartPlaces.RED_BACKDROP) {
                Imgproc.rectangle(input, leftRect, RED, 2);
                Imgproc.rectangle(input, centerRect, RED, 2);
                Imgproc.rectangle(input, rightRect, RED, 2);

                Core.extractChannel(leftCrop, leftCrop,0);
                Core.extractChannel(centerCrop, centerCrop,0);
                Core.extractChannel(rightCrop, rightCrop,0);

                Scalar leftAvg = Core.mean(leftCrop);
                Scalar centerAvg = Core.mean(centerCrop);
                Scalar rightAvg = Core.mean(rightCrop);

                double leftAvgRed = leftAvg.val[0];
                double centerAvgRed = centerAvg.val[0];
                double rightAvgRed = rightAvg.val[0];

                telemetry.addData("Left Red",leftAvgRed);
                telemetry.addData("Center Red",centerAvgRed);
                telemetry.addData("Right Red",rightAvgRed);


                double maxPercentRed = Math.min(Math.min(leftAvgRed, centerAvgRed), rightAvgRed);

                if (maxPercentRed == rightAvgRed) {
                    propPlacement = propPlaces.RIGHT;
                } else if (maxPercentRed == centerAvgRed) {
                    propPlacement = propPlaces.CENTER;
                } else if (maxPercentRed == leftAvgRed) {
                    propPlacement = propPlaces.LEFT;
                }
            } else if (robotStartPosition == robotStartPlaces.BLUE_HUMANPLAYER || robotStartPosition == robotStartPlaces.BLUE_BACKDROP){

                Core.extractChannel(leftCrop, leftCrop,2);
                Core.extractChannel(centerCrop, centerCrop,2);
                Core.extractChannel(rightCrop, rightCrop,2);

                Scalar leftAvg = Core.mean(leftCrop);
                Scalar centerAvg = Core.mean(centerCrop);
                Scalar rightAvg = Core.mean(rightCrop);

                double leftAvgBlue = leftAvg.val[0];
                double centerAvgBlue = centerAvg.val[0];
                double rightAvgBlue = rightAvg.val[0];

                double maxPercentBlue = Math.min(Math.min(leftAvgBlue, centerAvgBlue), rightAvgBlue);

                if (maxPercentBlue == leftAvgBlue) {
                    propPlacement = propPlaces.LEFT;
                } else if (maxPercentBlue == centerAvgBlue) {
                    propPlacement = propPlaces.RIGHT;
                } else if (maxPercentBlue == rightAvgBlue) {
                    propPlacement = propPlaces.CENTER;
                }
            }

            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar(0, 0, 255), 2);

            return input;
        }
    }

}


