package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@TeleOp(name = "Vision")
public class BlockVisionTuning extends OpMode {
    FtcDashboard dashboard;
    public OpenCvCamera phoneCam;
    MultipleTelemetry telemetry;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new BlockDetectorPipeline());
        phoneCam.startStreaming(1080, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.update();
    }


    private static List<MatOfPoint> getloc(Mat frame) {
        try {
            Mat crop = frame.clone();
            Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV);
            //Create Mask
            Mat mask = new Mat();
            Core.inRange(crop, new Scalar(30, 40, 10), new Scalar(70, 255, 255), mask);
            //Find Contours
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double maxArea = 0;
            List<MatOfPoint> biggest = new ArrayList<>();
            int index = -1;
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > maxArea) {
                    maxArea = area;
                    List<MatOfPoint> current = new ArrayList<>();
                    current.add(wrapper);
                    biggest = current;
                }
            }

            //Centroid setup
//            Moments mmnts = Imgproc.moments(mask, true);
            return biggest;
            //}
        } catch (Exception e) {
            return null;
        }
    }

    class BlockDetectorPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            telemetry.addData("In Camera", "");
            Imgproc.drawContours(input, getloc(input), -1, new Scalar(255, 0, 0), 5);
            Imgproc.putText(input, "FPS: " + phoneCam.getFps(), new Point(375, 100), 1, 2, new Scalar(0, 255, 0), 4);
            return input;
        }
    }
}