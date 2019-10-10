//package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;
//
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.RobotLibsV2.lib.utils.vision.PSVisionUtils;
//import org.firstinspires.ftc.teamcode.RobotLibsV2.lib.vision.UVC.UVCCamera;
//import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;
//import org.opencv.android.Utils;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//
//public class Camera implements Subsystem {
//    //custom camera object
//    UVCCamera camera;
//    OpMode opMode;
//    private int skyPos;
//
//    public Camera(OpMode mode) {
//        opMode = mode;
//    }
//
//    //load camera (init)
//    public void load(UVCCamera.Callback callback) {
//        //check if camera already exists
//        if (camera == null) {
//            // find camera that is plugged into usb hub
//            camera = UVCCamera.getCamera(callback);
//        }
//    }
//
//    //start camera stream
//    public void start() {
//        // check if camera exists
//        if (camera != null) {
//            //start camera stream
//            camera.start();
//        }
//    }
//
//    //stop camera stream
//    public void stop() {
//        //check if camera exists
//        if (camera != null) {
//            //stop
//            camera.stop();
//            //remove storage of camera
//            camera = null;
//        }
//    }
//
//    public Mat getSkyStone(Bitmap bm) {
//        Mat input = new Mat();
//        Mat hsvLeft = new Mat();
//        Mat hsvMiddle = new Mat();
//        Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
//        Utils.bitmapToMat(bmp32, input);
//        Rect rectCrop = new Rect(0, 120, 160, 120);
//        Mat leftMineral = input.submat(rectCrop);
//        rectCrop = new Rect(320, 120, 160, 120);
//        Mat middleMineral = input.submat(rectCrop);
//        Imgproc.cvtColor(leftMineral, hsvLeft, Imgproc.COLOR_RGB2HSV);
//        Imgproc.cvtColor(middleMineral, hsvMiddle, Imgproc.COLOR_RGB2HSV);
//        double leftYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvLeft, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "leftY");
//        double middleYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvMiddle, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "middleY");
//        skyPos = ((leftYellowArea > middleYellowArea && leftYellowArea > 200) ? 1 : ((middleYellowArea > 200) ? 2 : 3));
//        return null;
//    }
//
//    @Override
//    public void update() {
//
//    }
//}
