package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.FilterReader;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Auto extends OpMode {
    Robot robot;
    OpenCvCamera openCvCamera;
    AutoStates state;
    AllianceColors allianceColor = AllianceColors.RED;
    //Build Zone
    //0  L  0
    //1  O  1
    //2  A  2
    //3  D  3
    //4  I  4
    //5  N  5
    //   G
    final double[][] redQuarryStonePoses = {{-22, -28}, {-22, -36}, {-22, -44}, {-22, -52}, {-22, -60}, {-22, -68}};
    final double[][] blueQuarryStonePoses = {{22, -28}, {22, -36}, {22, -44}, {22, -52}, {22, -60}, {22, -68}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    static String opencvLoad = "";


    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private boolean foundationMoved = false;

    @Override
    public void init() {
        robot = new Robot(this);
        stickygamepad1 = new StickyGamepad(gamepad1);
        elapsedTime = new ElapsedTime();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    }

    @Override
    public void init_loop() {
        allianceColor = (stickygamepad1.x) ? AllianceColors.RED : AllianceColors.BLUE;
        telemetry.addData("Alliance Color", allianceColor);
        stickygamepad1.update();

    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
    }


    @Override
    public void loop() {
        switch (state) {
            case SKYSTONE_DETECT:
                robot.mecanumDrive.follower.followTrajectory(startToSkyStone(skyPos));
                state = AutoStates.PATH_TO_STONES;
                robot.depositLift.setTargetHeight(2);
                break;
            case PATH_TO_STONES:
                //lift lift to hover height
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.mecanumDrive.follower.followTrajectory(goIntoStones());
                    state = AutoStates.MOVE_INTO_STONES;

                }
                break;
            case MOVE_INTO_STONES:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()||skyPos==0) {
                    robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    robot.intake.setIntakePower(1);
                    state = AutoStates.STONE_PICK;
                }
                break;
            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.PI / 2, 0.3, 0);
                if (robot.depositLift.isStoneInBot()) {
                    //lift down and grab block
                    robot.depositLift.setTargetHeight(0.5);
                    robot.mecanumDrive.follower.followTrajectory(stonesToPlatform1());
                    state = AutoStates.PATH_TO_FOUNDATION;
                }
                //wait till stone in bot

                break;
            case PATH_TO_FOUNDATION:

                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.depositLift.grabStone();
                    robot.depositLift.setTargetHeight(4);
                    state = AutoStates.PLACE_STONE;
                    elapsedTime.reset();
                }
                break;
            case PLACE_STONE:

                if (elapsedTime.seconds()<0.75){
                    robot.depositLift.setExtensionPower(1);
                } else if (elapsedTime.seconds()<1) {
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds()<1.5){
                    robot.depositLift.setExtensionPower(-1);
                } else{
                    if(foundationMoved){
                        robot.mecanumDrive.follower.followTrajectory(platformToStones(1));//TODO Add logic
                        state = AutoStates.PATH_TO_STONES;
                    } else {
                        state = AutoStates.MOVE_FOUNDATION;
                        robot.mecanumDrive.platformGrab();
                        robot.mecanumDrive.follower.followTrajectory(movePlatform());
                        foundationMoved = true;
                    }
                }

                break;
            case MOVE_FOUNDATION:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.mecanumDrive.follower.followTrajectory(platformToStones(1));//TODO Add logic
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(2);
                }
                //grab foundation
                //PID to build zone
                //release foundation
                //loop back PATH_TO_STONES
                break;
            case PARK:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.IDLE;

                }
                //update follower
                //IDLE
                break;
            case IDLE:
                break;
        }
        if (time>28){
            state = AutoStates.PARK;
            robot.mecanumDrive.follower.followTrajectory(parkPath());
        }
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        robot.mecanumDrive.updatePoseEstimate();
        robot.depositLift.updateLiftPower(robot.depositLift.pid.update(robot.depositLift.getRelLiftHeight()));
        telemetry.update();
    }

    private Trajectory parkPath() {
        return null;
    }


    public Trajectory startToSkyStone(int skyStonePos) {
        if (0 != skyStonePos) {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 15, -33), new LinearInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 180))
                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 15, -33), new LinearInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 135))
                    .build();
        }

    }

    public Trajectory stonesToPlatform1() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                //spline to the side
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX(), robot.mecanumDrive.getPoseEstimate().getY() - 11), new LinearInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(),0))
                .splineTo(new Pose2d(0, -36), new ConstantInterpolator(0))
                .splineTo(new Pose2d(24, -36), new ConstantInterpolator(0))
                .splineTo(new Pose2d(24, -17), new ConstantInterpolator(0))
                .build();
    }

    public Trajectory movePlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX(), -63), new ConstantInterpolator(0))
                .addMarker(()->{robot.mecanumDrive.platformRelease(); return Unit.INSTANCE;})
                .build();
    }

    public Trajectory platformToStones(int stone) {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(-48, 0), new ConstantInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading()))
//                .addMarker(() -> {
//                    robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
//                    robot.intake.setIntakePower(1);
//                    return Unit.INSTANCE;
//                })
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 6, 33), new LinearInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 180))
                .build();
    }

    public Trajectory goIntoStones() {
        //TODO change pos of stones that have moved
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraintsSlow())
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX() + 6, robot.mecanumDrive.getPoseEstimate().getY()), new ConstantInterpolator(180))
                .build();
    }

    public enum AutoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, MOVE_INTO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum InitStates {
        CAMERA_START, SKYSTONE_DETECT, CAMERA_STOP
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
