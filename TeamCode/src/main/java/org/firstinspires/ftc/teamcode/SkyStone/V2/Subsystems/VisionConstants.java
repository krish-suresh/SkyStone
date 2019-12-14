package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class VisionConstants {
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
    public static Scalar HSV_LOW = new Scalar(10, 20, 70);
    public static Scalar HSV_HIGH = new Scalar(30, 255, 255);
    public static Rect rectCrop0 = new Rect(475, 300, 120, 60);
    public static Rect rectCrop1 = new Rect(335, 300, 120, 60);
    public static Rect rectCrop2 = new Rect(190, 300, 120, 60);
}
