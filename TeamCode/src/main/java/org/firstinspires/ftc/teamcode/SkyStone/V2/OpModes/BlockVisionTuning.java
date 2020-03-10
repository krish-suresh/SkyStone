package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.SkystoneDetectorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_WIDTH;

@TeleOp(name = "Vision")
@Disabled
public class BlockVisionTuning extends OpMode {
    FtcDashboard dashboard;
    public OpenCvCamera phoneCam;
    MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;
    OpenCvCamera webcam;
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
        telemetry.addData("Stonesize0", pipeline.stoneSizes[0]);
        telemetry.addData("Stonesize1", pipeline.stoneSizes[1]);
        telemetry.addData("Stonesize2", pipeline.stoneSizes[2]);
        telemetry.update();

    }

}