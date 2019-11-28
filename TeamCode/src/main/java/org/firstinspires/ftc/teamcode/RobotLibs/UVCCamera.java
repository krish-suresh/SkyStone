package org.firstinspires.ftc.teamcode.SkyStone.RobotLibs;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;

import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;

public class UVCCamera {
    final CameraManager cameraManager;
    final CameraName cameraName;
    final Executor threadPool = ThreadPool.newSingleThreadExecutor("UVCCamera");
    CameraCharacteristics characteristics;
    int imageFormatWanted = ImageFormat.YUY2;
    Camera camera;
    Size sizeWanted;
    UVCCamera.Callback frameCallback;
//    private int cameraMonitorViewId;
//    private ViewGroup cameraMonitorView;
//
//    AppUtil appUtil = AppUtil.getInstance();
//    Activity activity;

    public UVCCamera(CameraManager cameraManager, CameraName cameraName, UVCCamera.Callback frameCallback) {
        this.cameraManager = cameraManager;
        this.cameraName = cameraName;
        this.frameCallback = frameCallback;
//        this.activity = appUtil.getActivity();
//        Context context = AppUtil.getDefContext();
//        cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
    }

    public static UVCCamera getCamera(UVCCamera.Callback callback) {
        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        UVCCamera ret = null;
        for (CameraName cameraName : cameraManager.getAllWebcams()) {
            ret = new UVCCamera(cameraManager, cameraName, callback);
        }
        return ret;
    }

    public void start() {
//        if (cameraMonitorViewId == 0) {
//            cameraMonitorView = (ViewGroup) activity.findViewById(android.R.id.content);
//        } else {
//            cameraMonitorView = (LinearLayout) activity.findViewById(cameraMonitorViewId);
//        }
        //cameraMonitorView.addView(cameraView);
        Deadline deadline = new Deadline(10, TimeUnit.SECONDS);

//        Boolean value = cameraName.requestCameraPermission(deadline);

        cameraName.asyncRequestCameraPermission(AppUtil.getDefContext(), deadline, Continuation.create(threadPool, new Consumer<Boolean>() {
            @Override
            public void accept(Boolean permissionGranted) {
                if (permissionGranted) {
                    cameraManager.asyncOpenCameraAssumingPermission(cameraName, Continuation.create(threadPool, new Camera.StateCallbackDefault() {
                        @Override
                        public void onOpened( final Camera camera) {
                            UVCCamera.this.camera = camera;

                            characteristics = cameraName.getCameraCharacteristics();

                            if (Misc.contains(characteristics.getAndroidFormats(), imageFormatWanted)) {
                                sizeWanted = characteristics.getDefaultSize(imageFormatWanted);
                                try {
                                    camera.createCaptureSession(Continuation.create(threadPool, captureStateCallback));
                                } catch (CameraException e) {
                                }
                            } else {
//                                RobotLog.ee(TAG, "camera doesn't support desired format: 0x%02x", imageFormatWanted);
                            }
                        }

                        @Override
                        public void onClosed( Camera camera) {
//                            RobotLog.vv(TAG, "camera reports closed: %s", camera);
                        }
                    }), 10, TimeUnit.SECONDS);
                }
            }
        }));
    }

    CameraCaptureSession cameraCaptureSession;
    CameraCaptureSession.StateCallback captureStateCallback = new CameraCaptureSession.StateCallbackDefault() {

        @Override
        public void onConfigured( CameraCaptureSession session) {
            cameraCaptureSession = session;
            try {
                /** Indicate <em>how</em> we want to stream. */
                final CameraCaptureRequest cameraCaptureRequest = camera.createCaptureRequest(imageFormatWanted, sizeWanted, characteristics.getMaxFramesPerSecond(imageFormatWanted, sizeWanted));
                /** Start streaming! Flow continues in the captureCallback. */
                CameraCaptureSequenceId cameraCaptureSequenceId = session.startCapture(cameraCaptureRequest, new UVCCamera.WebcamCaptureCallback(cameraCaptureRequest, frameCallback), // callback, not continuation; avoids copying frame
                        Continuation.create(threadPool, new CameraCaptureSession.StatusCallback() {
                            @Override
                            public void onCaptureSequenceCompleted( CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
//                                RobotLog.vv(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                            }
                        }));
            } catch (CameraException e) {
//                RobotLog.ee(TAG, e, "error setting repeat capture request");
            }
        }

        @Override
        public void onClosed( CameraCaptureSession session) {
//            RobotLog.vv(TAG, "capture session reports closed: %s", session);
        }
    };

    public void stop() {
        cameraCaptureSession.stopCapture();
        cameraCaptureSession.close();
        camera.close();
    }

    public interface Callback {
        Bitmap onFrame(Bitmap bm);
    }

    public class WebcamCaptureCallback implements CameraCaptureSession.CaptureCallback {
        Bitmap bitmap;
        UVCCamera.Callback callback;
        boolean stop = false;

        public WebcamCaptureCallback(CameraCaptureRequest request, UVCCamera.Callback callback) {
            bitmap = request.createEmptyBitmap();
            this.callback = callback;
        }

        @Override
        public void onNewFrame( CameraCaptureSession session,  CameraCaptureRequest request,  CameraFrame cameraFrame) {
            cameraFrame.copyToBitmap(bitmap);
            Bitmap save = callback.onFrame(bitmap);
            if (save != null) VisionUtils.saveImageToFile(save, "frame_save", "/saved_images");

        }
    }
}
