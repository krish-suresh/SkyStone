package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.SkystoneDetectorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_WIDTH;

@TeleOp(name = "Vision")
public class BlockVisionTuning extends OpMode {
    FtcDashboard dashboard;
    public Camera camera;
    MultipleTelemetry telemetry;
    public void init() {
        dashboard = FtcDashboard.getInstance();
        camera = new Camera(this);

        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {


        telemetry.addData("Stone Pos", camera.getSkyPos(true));
        telemetry.update();
        dashboard.sendImage(camera.pipeline.imageSend);

    }

}