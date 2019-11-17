package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import kotlin.Unit;

//TODO Add in comments for what each position is on the field
@Autonomous(name = "Auto")
public class Auto extends OpMode {
    Robot robot;
    AutoStates state = AutoStates.SKYSTONE_DETECT;
    AllianceColors allianceColor = AllianceColors.RED;
    Camera camera;
    //Skystone positions 0-5:
    //Build Zone
    //0  L  0
    //1  O  1
    //2  A  2
    //3  D  3
    //4  I  4
    //5  N  5
    //   G  Quarry

    final double[][] redQuarryStonePoses = {{ -28,-22}, { -36,-22}, { -44,-22}, { -52,-22}, {-60,-22}, {-68,-22}};
    final double[][] blueQuarryStonePoses = {{22, -28}, {22, -36}, {22, -44}, {22, -52}, {22, -60}, {22, -68}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    List<Integer> quarryStones = Arrays.asList(0, 1, 2, 3, 4, 5);

    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private boolean foundationMoved = false;
    private int currentStone;

    @Override
    public void init() {
        robot = new Robot(this);
        camera = new Camera(this);
        stickygamepad1 = new StickyGamepad(gamepad1);
        elapsedTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
    }

    @Override
    public void init_loop() {
        allianceColor = (stickygamepad1.x) ? AllianceColors.BLUE : AllianceColors.RED;
        telemetry.addData("Alliance Color", allianceColor);
        stickygamepad1.update();
        skyPos = camera.getSkyPos();
        telemetry.addData("skyPos", skyPos);
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
                currentStone = skyPos;
                state = AutoStates.PATH_TO_STONES;
//                robot.depositLift.setTargetHeight(2);
                break;
            case PATH_TO_STONES:
                robot.mecanumDrive.setDriveSignal(robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate()));
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.mecanumDrive.follower.followTrajectory(goIntoStones());
//                    state = AutoStates.MOVE_INTO_STONES;
                    state = AutoStates.IDLE;
                }
                break;
            case MOVE_INTO_STONES:
                robot.mecanumDrive.setDriveSignal(robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate()));
                if (!robot.mecanumDrive.follower.isFollowing() || skyPos == 0) {
                    if (quarryStones.size() < 5) {
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                    } else {
                        robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    }
                    robot.intake.setIntakePower(0.8);
                    state = AutoStates.STONE_PICK;
                }
                break;
            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.PI / 2, 0.3, 0);
                if (robot.depositLift.isStoneInBot()) {
                    robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
                    robot.depositLift.setTargetHeight(0.5);
                    robot.intake.setIntakePower(0);
                    if (foundationMoved) {
                        robot.mecanumDrive.follower.followTrajectory(stonesToPlatform());
                    } else {
                        robot.mecanumDrive.follower.followTrajectory(stonesToPlatform1st());
                    }
                    state = AutoStates.PATH_TO_FOUNDATION;
                }

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

                if (elapsedTime.seconds() < 0.75) {
                    robot.depositLift.setExtensionPower(1);
                } else if (elapsedTime.seconds() < 1) {
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 1.5) {
                    robot.depositLift.setExtensionPower(-1);
                } else {
                    quarryStones.remove(quarryStones.indexOf(currentStone));
                    if (foundationMoved) {
                        currentStone = quarryStones.get(0);
                        robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
                        state = AutoStates.PATH_TO_STONES;
                    } else {
                        state = AutoStates.MOVE_FOUNDATION;

                        robot.mecanumDrive.platformGrab();//TODO ADD WAIT AFTER THIS

                        robot.mecanumDrive.follower.followTrajectory(movePlatform());
                        foundationMoved = true;
                    }
                }
                break;
            case MOVE_FOUNDATION:
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    currentStone = skyPos + 3;
                    robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(2);
                }
                break;
            case PARK:
                robot.depositLift.setTargetHeight(0.5);
                robot.mecanumDrive.follower.update(robot.mecanumDrive.getPoseEstimate());
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.IDLE;

                }
                break;
            case IDLE:
                robot.mecanumDrive.setMotorPowers(0,0,0,0);
                requestOpModeStop();
                break;
        }
        if (time > (30.0 - parkPath().duration())) {
            state = AutoStates.PARK;
            robot.mecanumDrive.follower.followTrajectory(parkPath());
        }
        telemetry.addData("Enc Pos",robot.mecanumDrive.getWheelPositions());
        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        robot.mecanumDrive.updatePoseEstimate();
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getRelLiftHeight() - robot.depositLift.liftStartCal));
        telemetry.update();
    }

    private Trajectory parkPath() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, -36), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 0))
                .build();
    }


    public Trajectory startToSkyStone(int skyStonePos) {
        if (0 != skyStonePos) {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 15, -35), new SplineInterpolator(Math.PI/2, 0))
                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 8, -35), new SplineInterpolator(Math.PI/2,Math.PI/4))
                    .build();
        }
//        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints()).forward(10).build();
    }

    public Trajectory stonesToPlatform1st() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                //spline to the side
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX(), robot.mecanumDrive.getPoseEstimate().getY() - 11), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 0))
                .splineTo(new Pose2d(0, -36), new ConstantInterpolator(0))
                .splineTo(new Pose2d(48, -32), new SplineInterpolator(0, Math.toRadians(270)))

                .build();
    }

    public Trajectory stonesToPlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                //spline to the side
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX(), -35), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), 0))
                .splineTo(new Pose2d(0, -36), new ConstantInterpolator(0))
                .splineTo(new Pose2d(43, -45), new ConstantInterpolator(0))//TODO FIND REAL POS
                .build();
    }

    public Trajectory movePlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(36, -45, 0),new TangentInterpolator())
                .splineTo(new Pose2d(43, -45), new ConstantInterpolator(0))
                .addMarker(() -> {
                    robot.mecanumDrive.platformRelease();
                    return Unit.INSTANCE;
                })
                .build();
    }

    public Trajectory platformToStones(int stone) {
        double turnAngleForCollect = 0 != stone ? 0 : Math.toRadians(45);  //?
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, -36), new ConstantInterpolator(0))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, -33), new ConstantInterpolator(0))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, quarryStonePoses[stone][1] - 10), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), turnAngleForCollect))
                .build();

    }

    public Trajectory goIntoStones() {

        //if we drive into the side of a stone adjust the array to reflect the new position TODO change for 3 stones behind
        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += Robot.ROBOT_WIDTH / 2;
        } else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += Robot.ROBOT_WIDTH / 2;
            quarryStonePoses[currentStone - 2][1] += Robot.ROBOT_WIDTH / 2;
        }

        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraintsSlow())
                .splineTo(new Pose2d(robot.mecanumDrive.getPoseEstimate().getX() + 6, robot.mecanumDrive.getPoseEstimate().getY()), new ConstantInterpolator(0))
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
