package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.VisionConstants.IMAGE_WIDTH;

public class Camera implements Subsystem {
    public OpenCvCamera webcam;
    public SkystoneDetectorPipeline pipeline;
    OpMode opMode;
    public Camera(OpMode mode){
        opMode = mode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }
    @Override
    public void update() {

    }

    public int getSkyPos(boolean allianceColorisRed) {
        int skyPos = pipeline.getSkyPos();
        if (!allianceColorisRed) {
            return (skyPos == 0 ? 2 : (skyPos == 2) ? 0 : 1);
        }
        return skyPos;
    }
}
