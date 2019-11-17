package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.RobotLibs.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLibs.VisionUtils;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_WIDTH;

public class Camera implements Subsystem {
    public OpenCvCamera phoneCam;
    SkystoneDetectorPipeline pipeline;
    OpMode opMode;
    public Camera(OpMode mode){
        opMode = mode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }
    @Override
    public void update() {

    }

    public int getSkyPos() {
        return pipeline.getSkyPos();
    }
}
