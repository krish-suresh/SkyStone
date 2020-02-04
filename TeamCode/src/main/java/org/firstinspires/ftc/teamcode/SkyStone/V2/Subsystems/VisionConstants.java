package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class VisionConstants {
    public static int IMAGE_WIDTH = 640;
    public static int IMAGE_HEIGHT = 480;
    public static Scalar HSV_LOW = new Scalar(10, 20, 70);
    public static Scalar HSV_HIGH = new Scalar(30, 255, 255);
    public static Rect rectCrop0 = new Rect(20, 380, 80, 40);//475
    public static Rect rectCrop1 = new Rect(160, 380, 80, 40);//335
    public static Rect rectCrop2 = new Rect(240, 380, 80, 40);//475
    public static Rect rectCrop3 = new Rect(120, 380, 80, 40);//335

    // public static Rect rectCrop2 = new Rect(190, 300, 120, 60);
}
