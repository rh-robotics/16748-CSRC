package org.firstinspires.ftc.teamcode.auton.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.TestFramePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Autonomous OpMode to test scanning signal sleeves
 */
@Autonomous(name = "Camera Test Frame")
public class TestFrame extends OpMode {
    OpenCvCamera camera;
    OpenCvCamera cameraUneditedFootage;

    TestFramePipeline testFramePipeline;
    @Override
    public void init() {

        /* Activating the camera monitor view on robot controller phone. */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        /* All img processing happens within the pipeline. */
        testFramePipeline = new TestFramePipeline();

        camera.setPipeline(testFramePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                /* 16:9 aspect ratio.
                 * Streaming over 480p (640x480) limits our FPS to less than 30. */
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            /* If camera cannot be opened */
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException (
                        "Camera could not be opened."
                );
            }
        });
    }

    @Override
    public void loop () {}
}