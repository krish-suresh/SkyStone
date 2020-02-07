//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class VisionConstants {

    public static final int IMAGE_WIDTH = 640;      // resolution of webcam
    public static final int IMAGE_HEIGHT = 480;

    public static Scalar HSV_LOW = new Scalar(10, 20, 70);      // low/high values used in masks
    public static Scalar HSV_HIGH = new Scalar(30, 255, 255);

    public static Rect redStone0 = new Rect(475, 300, 120, 60);     // positions looked at in the screen to determine masks
    public static Rect redStone1 = new Rect(335, 300, 120, 60);
    public static Rect redStone2 = new Rect(190, 300, 120, 60);

    // TODO: tune xywh
    public static Rect blueStone0 = new Rect(475, 300, 120, 60);
    public static Rect blueStone1 = new Rect(335, 300, 120, 60);
    public static Rect blueStone2 = new Rect(190, 300, 120, 60);
}
