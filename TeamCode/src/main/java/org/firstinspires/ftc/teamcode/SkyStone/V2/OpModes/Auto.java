package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.RobotLibs.UVCCamera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
import org.opencv.android.OpenCVLoader;


public class Auto extends OpMode implements UVCCamera.Callback{
    Robot robot;
    AutoStates state;
    AllianceColors allianceColor = AllianceColors.RED;

    private int skyPos;
    static String opencvLoad = "";
    static {
        if (!OpenCVLoader.initDebug()) {
            opencvLoad = "Error Loading!";
        } else {
            opencvLoad = "Loaded Successfully!";
        }
    }

    private StickyGamepad stickygamepad1;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.camera.load(this);
        robot.camera.start();
        stickygamepad1=new StickyGamepad(gamepad1);

    }

    @Override
    public void init_loop(){
        allianceColor = (stickygamepad1.x)?AllianceColors.BLUE:AllianceColors.RED;
        telemetry.addData("Alliance Color", allianceColor);
        stickygamepad1.update();
    }
    @Override
    public void loop() {
        switch (state) {
            case SKYSTONE_DETECT:
                robot.mecanumDrive.follower.followTrajectory(robot.mecanumDrive.startToSkyStone(robot.camera.skyPos));
                state = AutoStates.PATH_TO_STONES;
                break;
            case PATH_TO_STONES:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getRobotPos());
                //Lift to hover height
                //extend out
                //rotate block
                break;
            case STONE_PICK:
                //lift down
                //grab
                //lift up
                robot.mecanumDrive.follower.followTrajectory(robot.mecanumDrive.stonesToPlatform1());
                break;
            case PATH_TO_FOUNDATION:
                //rotate block
                //extend in
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getRobotPos());
                break;
            case PLACE_STONE:
                //lift up
                //extend out
                //lift down
                // releaseblock
                //if foundation moved --> loop back PATH_TO_STONES --> if time>25s set path to park
                break;
            case MOVE_FOUNDATION:
                //grab foundation
                //PID to build zone
                //release foundation
                //loop back PATH_TO_STONES
                break;
            case PARK:
                //update follower
                //IDLE
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        if(state == AutoStates.SKYSTONE_DETECT) {
            robot.camera.getSkyStone(bm);
        }
        return null;
    }

    public enum AutoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }
    public enum InitStates {
        CAMERA_START, SKYSTONE_DETECT, CAMERA_STOP
    }
    public enum AllianceColors{

        RED, BLUE
    }

}
