package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

public class CustomElementPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat original = input.clone();

        Mat zone1 = input.submat(new Rect(0, 180, 50, 50));
        Mat zone2 = input.submat(new Rect(316, 180, 50, 50));
        Mat zone3 = input.submat(new Rect(590, 180, 50, 50));



        return input;
    }
}
