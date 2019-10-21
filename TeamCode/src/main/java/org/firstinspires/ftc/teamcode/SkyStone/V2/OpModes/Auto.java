package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.UVCCamera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
import org.opencv.android.OpenCVLoader;


public class Auto extends OpMode implements UVCCamera.Callback{
    Robot robot;
    autoStates state;
    private int skyPos;
    static String opencvLoad = "";
    static {
        if (!OpenCVLoader.initDebug()) {
            opencvLoad = "Error Loading!";
        } else {
            opencvLoad = "Loaded Successfully!";
        }
    }
    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        switch (state) {
            case SKYSTONE_DETECT:
                break;
            case PATH_TO_STONES:
                break;
            case STONE_PICK:
                break;
            case PATH_TO_FOUNDATION:
                break;
            case PLACE_STONE:
                break;
            case MOVE_FOUNDATION:
                break;
            case PARK:
                break;
        }
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        if(state == autoStates.SKYSTONE_DETECT) {
            robot.camera.getSkyStone(bm);
        }
        return null;
    }

    public enum autoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK
    }
    public enum initStates {
        CAMERA_START, SKYSTONE_DETECT, CAMERA_STOP
    }
}
