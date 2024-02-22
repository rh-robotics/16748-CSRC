package org.firstinspires.ftc.teamcode.auton.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.vision.SleeveDetection;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionCamera extends OpMode {
    private Hardware robot;
    SleeveDetection sleeveDetection = new SleeveDetection(145, 168, 30, 50);
    public OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
    String webcamName = "webcam1";
    int colorDetected = 0;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(20, 20, 20, 20);
        webcam1.setPipeline(sleeveDetection);
    }

    @Override
    public void loop() {

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
               webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
               colorDetected = sleeveDetection.getColor();

               if (colorDetected == 2) {
                   telemetry.addData("Color: ", "Blue");
               } else if (colorDetected == 1) {
                   telemetry.addData("Color: ", "Red");
               } else {
                   telemetry.addData("Color: ", "None Detected");
               }
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }
}
