//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.Subsystem.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.VisionConstants.IMAGE_WIDTH;

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

    // make it use an int[] from pipeline and return the right stone by color
    public int getSkyPos(boolean allianceColorisRed) {
        int skyPos = pipeline.getSkyPos();
        if (!allianceColorisRed) {
            return (skyPos == 0 ? 2 : (skyPos == 2) ? 0 : 1);
        }
        return skyPos;
    }
}
