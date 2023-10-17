package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class UneditedFootagePipeline extends OpenCvPipeline {
    /* All image processing happens here. */
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}