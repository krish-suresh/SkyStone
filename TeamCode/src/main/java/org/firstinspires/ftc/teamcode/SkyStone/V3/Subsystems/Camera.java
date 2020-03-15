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
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    @Override
    public void update() {

    }


    public int getSkyPos(boolean allianceColorisRed) {
        // pipeline.getSkyPos returns an int[] with {redSkyPos, blueSkyPos}
        int skyPos = pipeline.getSkyPos()[allianceColorisRed ? 0 : 1];    // choose which pos to use
        return skyPos;
    }
}
