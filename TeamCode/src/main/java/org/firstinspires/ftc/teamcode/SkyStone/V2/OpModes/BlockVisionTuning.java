package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
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
    //    FtcDashboard dashboard;
    public OpenCvCamera phoneCam;
    //    MultipleTelemetry telemetry;
    private boolean tempDown = true;
    private boolean tempUp = true;
    private boolean tempDDown = true;
    private boolean tempDUp = true;
    StickyGamepad stickyGamepad;
    private int HL = 10;
    private int SL = 20;
    private int VL = 70;
    private int HH = 30;
    private int SH = 255;
    private int VH = 255;
    public int targetLevel;
    public int changeVal = 0;

    public void init() {
//        dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new BlockDetectorPipeline());
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        stickyGamepad = new StickyGamepad(gamepad1);

    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        stickyGamepad.update();
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.addData("modval",targetLevel);
        telemetry.addData("HSV LOW",""+HL+" "+SL+" "+VL);
        telemetry.addData("HSV HIGH",""+HH+" "+SH+" "+VH);

        //set hue/saturation/value from d pad
        if (stickyGamepad.right_bumper == tempUp) {
            tempUp = !tempUp;
            targetLevel++;
        } else if (stickyGamepad.left_bumper == tempDown) {
            tempDown = !tempDown;
            targetLevel--;
        }
        if (stickyGamepad.dpad_up == tempDUp) {
            tempDUp = !tempDUp;
            changeVal = 10;
        } else if (stickyGamepad.dpad_down == tempDDown) {
            tempDDown = !tempDDown;
            changeVal = -10;
        }
        targetLevel = Range.clip(targetLevel, 0, 5);
        switch (targetLevel) {
            case 0:
                HL+=changeVal;
                break;
            case 1:
                SL+=changeVal;
                break;
            case 2:
                VL+=changeVal;
                break;
            case 3:
                HH+=changeVal;
                break;
            case 4:
                SH+=changeVal;
                break;
            case 5:
                VH+=changeVal;
                break;

        }
        changeVal = 0;
        telemetry.update();

    }


    private  List<MatOfPoint> getloc(Mat frame) {
        try {
            Mat crop = frame.clone();
            Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV);
            //Create Mask
            Mat mask = new Mat();
            Core.inRange(crop, new Scalar(HL, SL, VL), new Scalar(HH, SH, VH), mask);
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
            Imgproc.drawContours(input, getloc(input), -1, new Scalar(255, 0, 0), 5);
            return input;
        }
    }
}