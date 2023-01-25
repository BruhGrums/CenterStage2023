// This is just a proof of concept
// As the name implies, it does nothing

package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class EmptyPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

}
