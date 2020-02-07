//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;


import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.RobotLibs.VisionUtils;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.HSV_HIGH;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.HSV_LOW;

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redStone0;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redStone1;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redStone2;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueStone0;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueStone1;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueStone2;

public class SkystoneDetectorPipeline extends OpenCvPipeline {
    private int skyPos = 0;
    public double[] stoneSizes = {0, 0, 0};
    public Bitmap imageSend;

    @Override
    public Mat processFrame(Mat input) {
        Mat stone0 = input.submat(blueStone0);
        Mat stone1 = input.submat(blueStone1);
        Mat stone2 = input.submat(blueStone2);
        stoneSizes[0] = VisionUtils.maskSizeInMat(stone0, HSV_LOW, HSV_HIGH);
        stoneSizes[1] = VisionUtils.maskSizeInMat(stone1, HSV_LOW, HSV_HIGH);
        stoneSizes[2] = VisionUtils.maskSizeInMat(stone2, HSV_LOW, HSV_HIGH);

        double lowest = stoneSizes[0];
        skyPos = 0;
        for (int i = 0; i < stoneSizes.length; i++) {
            if (lowest > stoneSizes[i]) {
                skyPos = i;
                lowest = stoneSizes[i];
            }
        }

        switch (skyPos) {
            case 0:
                Imgproc.rectangle(input, blueStone0, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, blueStone1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueStone2, new Scalar(255, 0, 0), 3);
                break;
            case 1:
                Imgproc.rectangle(input, blueStone0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueStone1, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, blueStone2, new Scalar(255, 0, 0), 3);
                break;
            case 2:
                Imgproc.rectangle(input, blueStone0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueStone1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueStone2, new Scalar(0, 255, 0), 3);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + skyPos);
        }

        imageSend = VisionUtils.matToBitmap(input);
        return input;
    }
    public int getSkyPos(){
        return skyPos;
    }
}

