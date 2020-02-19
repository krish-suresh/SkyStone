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

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redCrop0;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redCrop1;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.redCrop2;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueCrop0;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueCrop1;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.blueCrop2;



public class SkystoneDetectorPipeline extends OpenCvPipeline {

    private int redSkyPos = 0;
    private int blueSkyPos = 0;
    public double[] redStoneSizes = {0, 0, 0};
    public double[] blueStoneSizes = {0, 0, 0};

    public Bitmap imageSend;


    @Override
    public Mat processFrame(Mat input) {

        // find blue skyPos

        // make 3 Mats based on the crops from VisionConstants
        Mat blueStone0 = input.submat(blueCrop0);
        Mat blueStone1 = input.submat(blueCrop1);
        Mat blueStone2 = input.submat(blueCrop2);

        // find the "size" of each crop (aka how much yellow is in each)
        blueStoneSizes[0] = VisionUtils.maskSizeInMat(blueStone0, HSV_LOW, HSV_HIGH);
        blueStoneSizes[1] = VisionUtils.maskSizeInMat(blueStone1, HSV_LOW, HSV_HIGH);
        blueStoneSizes[2] = VisionUtils.maskSizeInMat(blueStone2, HSV_LOW, HSV_HIGH);

        // skystone is in the crop with the least yellow
        double blueLowest = blueStoneSizes[0];
        blueSkyPos = 0;
        for (int i = 0; i < blueStoneSizes.length; i++) {
            if (blueLowest > blueStoneSizes[i]) {
                blueSkyPos = i;
                blueLowest = blueStoneSizes[i];
            }
        }

        // draw a rectangle in green around the blueSkyPos and in red around each normal stone
        switch (blueSkyPos) {
            case 0:
                Imgproc.rectangle(input, blueCrop0, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, blueCrop1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueCrop2, new Scalar(255, 0, 0), 3);
                break;
            case 1:
                Imgproc.rectangle(input, blueCrop0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueCrop1, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, blueCrop2, new Scalar(255, 0, 0), 3);
                break;
            case 2:
                Imgproc.rectangle(input, blueCrop0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueCrop1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, blueCrop2, new Scalar(0, 255, 0), 3);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + blueSkyPos);
        }



        // find red skyPos

        // make 3 Mats based on the crops from VisionConstants
        Mat redStone0 = input.submat(redCrop0);
        Mat redStone1 = input.submat(redCrop1);
        Mat redStone2 = input.submat(redCrop2);

        // find the "size" of each crop (aka how much yellow is in each)
        redStoneSizes[0] = VisionUtils.maskSizeInMat(redStone0, HSV_LOW, HSV_HIGH);
        redStoneSizes[1] = VisionUtils.maskSizeInMat(redStone1, HSV_LOW, HSV_HIGH);
        redStoneSizes[2] = VisionUtils.maskSizeInMat(redStone2, HSV_LOW, HSV_HIGH);

        // skystone is in the crop with the least yellow
        double redLowest = redStoneSizes[0];
        redSkyPos = 0;
        for (int i = 0; i < redStoneSizes.length; i++) {
            if (redLowest > redStoneSizes[i]) {
                redSkyPos = i;
                redLowest = redStoneSizes[i];
            }
        }

        // draw a rectangle in green around the redSkyPos and in red around each normal stone
        switch (redSkyPos) {
            case 0:
                Imgproc.rectangle(input, redCrop0, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, redCrop1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, redCrop2, new Scalar(255, 0, 0), 3);
                break;
            case 1:
                Imgproc.rectangle(input, redCrop0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, redCrop1, new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(input, redCrop2, new Scalar(255, 0, 0), 3);
                break;
            case 2:
                Imgproc.rectangle(input, redCrop0, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, redCrop1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, redCrop2, new Scalar(0, 255, 0), 3);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + redSkyPos);
        }

        imageSend = VisionUtils.matToBitmap(input);
        return input;
    }


    /**
     * Takes the values from processFrame() and returns them for the Camera class
     * @return the two skystone positions
     */
    public int[] getSkyPos(){
        return new int[] {redSkyPos, blueSkyPos};
    }
}

