package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.VisionUtils;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.SkystoneDetectorPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.HSV_HIGH;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.HSV_LOW;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_WIDTH;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.rectCrop0;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.rectCrop1;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.rectCrop2;

@Config
@TeleOp(name = "Vision")
public class BlockVisionTuning extends OpMode {
    FtcDashboard dashboard;
    public OpenCvCamera phoneCam;
    MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    public void init() {
        dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        dashboard.sendImage(pipeline.imageSend);
        telemetry.addData("Stone Pos", pipeline.getSkyPos());
        telemetry.update();

    }

}