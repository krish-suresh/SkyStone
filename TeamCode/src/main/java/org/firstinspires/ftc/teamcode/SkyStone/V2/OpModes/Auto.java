package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;


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

    @Override
    public void init() {
        robot = new Robot(this);
        stickygamepad1 = new StickyGamepad(gamepad1);
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
                break;
            case PATH_TO_STONES:
                //lift lift to hover height
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getRobotPos());
                if (!robot.mecanumDrive.follower.isFollowing()){
                    robot.mecanumDrive.follower.followTrajectory(goIntoStones());
                    state = AutoStates.MOVE_INTO_STONES;

                }
                    break;
            case MOVE_INTO_STONES:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getRobotPos());
                if (!robot.mecanumDrive.follower.isFollowing()){
                    robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    robot.intake.setIntakePower(1);
                    state = AutoStates.STONE_PICK;
                }
                break;
            case STONE_PICK:
                if (robot.depositLift.isStoneInBot()){
                    //lift down and grab block
                }
                //wait till stone in bot
                robot.mecanumDrive.follower.followTrajectory(stonesToPlatform1());
                state = AutoStates.PATH_TO_FOUNDATION;
                break;
            case PATH_TO_FOUNDATION:

                robot.mecanumDrive.follower.update(robot.mecanumDrive.getRobotPos());
                if (!robot.mecanumDrive.follower.isFollowing()){
                    robot.mecanumDrive.follower.followTrajectory(stonesToPlatform1());
                    state = AutoStates.STONE_PICK;
                }
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
        telemetry.addData("Robot Pos", robot.mecanumDrive.getRobotPos());
        robot.mecanumDrive.updateOdo();
        telemetry.update();
    }



    public Trajectory startToSkyStone(int skyStonePos) {

        return new TrajectoryBuilder(robot.mecanumDrive.getRobotPos(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 6, 33), new LinearInterpolator(robot.mecanumDrive.getRobotPos().getHeading(), 180))
                .build();

    }

    public Trajectory stonesToPlatform1() {
        return new TrajectoryBuilder(robot.mecanumDrive.getRobotPos(), robot.mecanumDrive.getConstraints())
                //spline to the side
                .splineTo(new Pose2d(0, -36), new ConstantInterpolator(robot.mecanumDrive.getRobotPos().getHeading()))
                .splineTo(new Pose2d(24, -17), new ConstantInterpolator(robot.mecanumDrive.getRobotPos().getHeading()))
                .build();
    }

    public Trajectory movePlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getRobotPos(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, -63), new ConstantInterpolator(robot.mecanumDrive.getRobotPos().getHeading()))
                .build();
    }

    public Trajectory platformToStones(int stone) {
        return new TrajectoryBuilder(robot.mecanumDrive.getRobotPos(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(-48, 0), new ConstantInterpolator(robot.mecanumDrive.getRobotPos().getHeading()))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 6, 33), new LinearInterpolator(robot.mecanumDrive.getRobotPos().getHeading(), 180))
                .build();
    }

    public Trajectory goIntoStones() {
        //TODO change pos of stones that have moved
        return new TrajectoryBuilder(robot.mecanumDrive.getRobotPos(), robot.mecanumDrive.getConstraintsSlow())
                .splineTo(new Pose2d(robot.mecanumDrive.getRobotPos().getX() + 6, robot.mecanumDrive.getRobotPos().getY()), new ConstantInterpolator(180))
                .build();
    }

    public enum AutoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, MOVE_INTO_STONES,STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum InitStates {
        CAMERA_START, SKYSTONE_DETECT, CAMERA_STOP
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
