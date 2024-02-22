package org.firstinspires.ftc.teamcode.auton.vision;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.vision.SleeveDetection;
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

@Autonomous(name = "Color Detection Cam")
public class ColorDetectionCamera extends OpMode {
    public OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
    String webcamName = "webcam1";
    int sectorDetected = 0;

    enum propPlaces {
    Right,
    Center,
    Left
    }

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        webcam1.setPipeline(new ColorDetection());
    }

    @Override
    public void loop() {

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

    public class ColorDetection extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat YCbCr = new Mat();
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            // Define sectors left, center, and right sectors
            Rect leftRect = new Rect(1, 1, 104, 239);
            Rect centerRect = new Rect(105, 1, 109, 239);
            Rect rightRect = new Rect(215, 1, 104, 239);

            Mat leftCrop = YCbCr.submat(leftRect);
            Mat centerCrop = YCbCr.submat(centerRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            // Greyscale
            Mat leftGray = new Mat();
            Mat centerGray = new Mat();
            Mat rightGray = new Mat();
            Imgproc.cvtColor(leftCrop, leftGray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(centerCrop, centerGray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(rightCrop, rightGray, Imgproc.COLOR_BGR2GRAY);

            // Isolate the red and blue
            int redThreshold = 100;
            int blueThreshold = 100;
            Mat leftBinaryRed = new Mat();
            Mat centerBinaryRed = new Mat();
            Mat rightBinaryRed = new Mat();
            Mat leftBinaryBlue = new Mat();
            Mat centerBinaryBlue = new Mat();
            Mat rightBinaryBlue = new Mat();
            Imgproc.threshold(leftGray, leftBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(centerGray, centerBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(rightGray, rightBinaryRed, redThreshold, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(leftGray, leftBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(centerGray, centerBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(rightGray, rightBinaryBlue, blueThreshold, 255, Imgproc.THRESH_BINARY);

            boolean isRedLeft = Core.countNonZero(leftBinaryRed) > 0;
            boolean isRedCenter = Core.countNonZero(centerBinaryRed) > 0;
            boolean isRedRight = Core.countNonZero(rightBinaryRed) > 0;
            boolean isBlueLeft = Core.countNonZero(leftBinaryBlue) > 0;
            boolean isBlueCenter = Core.countNonZero(centerBinaryBlue) > 0;
            boolean isBlueRight = Core.countNonZero(rightBinaryBlue) > 0;

            if (isRedLeft || isBlueLeft) {
                propPlaces propPlacement = propPlaces.Left;
            } else if (isRedCenter || isBlueCenter) {
                propPlaces propPlacement = propPlaces.Center;
            } else if (isRedRight || isBlueRight) {
                propPlaces propPlacement = propPlaces.Right;
            }


            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar(0, 0, 255), 2);

            return input;
        }
    }

}


