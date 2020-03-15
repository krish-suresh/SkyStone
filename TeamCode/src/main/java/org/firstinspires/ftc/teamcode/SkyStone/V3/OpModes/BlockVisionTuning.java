//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.SkystoneDetectorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.IMAGE_WIDTH;

@TeleOp(name = "Vision")
public class BlockVisionTuning extends OpMode {
    FtcDashboard dashboard;
    public OpenCvCamera phoneCam;
    MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;
    OpenCvCamera webcam;
    public void init() {
        dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        dashboard.sendImage(pipeline.imageSend);

        telemetry.addData("Stone Pos", pipeline.getSkyPos());
        telemetry.addData("Stonesize0", pipeline.redStoneSizes[0]);
        telemetry.addData("Stonesize1", pipeline.redStoneSizes[1]);
        telemetry.addData("Stonesize2", pipeline.redStoneSizes[2]);
        telemetry.update();

    }

}