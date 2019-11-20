package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
//TODO Retune the turning/trackwidth and make movement 30ips
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

    final double[][] redQuarryStonePoses = {{-28, -22}, {-36, -22}, {-44, -22}, {-52, -22}, {-60, -22}, {-68, -22}};
    final double[][] blueQuarryStonePoses = {{22, -28}, {22, -36}, {22, -44}, {22, -52}, {22, -60}, {22, -68}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    ArrayList<Integer> quarryStones = new ArrayList<>();

    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private boolean foundationMoved = false;
    private int currentStone;
    private boolean allianceColorisRed = true;

    @Override
    public void init() {
        robot = new Robot(this);
        camera = new Camera(this);
        stickygamepad1 = new StickyGamepad(gamepad1);
        elapsedTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));
    }

    @Override
    public void init_loop() {
        allianceColor = (stickygamepad1.x) ? AllianceColors.BLUE : AllianceColors.RED;
        telemetry.addData("Alliance Color", allianceColor);
        stickygamepad1.update();
        skyPos = camera.getSkyPos();
        telemetry.addData("skyPos", skyPos);
        if (allianceColor == AllianceColors.RED) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, -63, Math.PI / 2));// Red start pos
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, 63, Math.PI * 3 / 2));// Red start pos
        }

    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
        allianceColorisRed = allianceColor == AllianceColors.RED;//This is used to assign positions for the splines based on alliance
    }


    @Override
    public void loop() {
        switch (state) {
            case SKYSTONE_DETECT://This is the starting state where we create the traj to the correct stone and lift the height a bit
                robot.mecanumDrive.follower.followTrajectory(startToSkyStone(skyPos));
                currentStone = skyPos;
                state = AutoStates.PATH_TO_STONES;
                camera.phoneCam.stopStreaming();
                break;
            case PATH_TO_STONES://This follows the path to right next to the stone, if the stone is 0 then we skip the Move into Stone cus we need to collect at an angle
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.mecanumDrive.follower.followTrajectory(goIntoStones());
                    robot.depositLift.setTargetHeight(4);
                    if (currentStone != 0) {
                        state = AutoStates.MOVE_INTO_STONES;
                    } else {
                        state = AutoStates.STONE_PICK;
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                        robot.intake.setIntakePower(0.8);//Since move into stones is skipped, we still need to update the intake stuff
                    }
                }
                break;
            case MOVE_INTO_STONES://If the current stone is not 0 we need to straffe into the quarry to the correct pos to be able to drive straight into the stones
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    if (quarryStones.size() < 5) {//this makes it so that after we have collected both skystones we have the collector in the middle pos to give us more tol when we collect
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                    } else {
                        robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    }
                    robot.intake.setIntakePower(0.5);//sets intake power
                    state = AutoStates.STONE_PICK;
                }
                break;
            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.PI, 0.15, 0);//This make the robot drive forward slowly
                //TODO add smart collection
                if (robot.depositLift.isStoneInBot()) {//Once the stone is in the robot we can stop the collection and lower the lift
                    robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
                    robot.depositLift.setTargetHeight(0);//lowers lift
                    robot.intake.setIntakePower(0);//stop collecting
                    robot.mecanumDrive.stopDriveMotors();//stops DT
                    state = AutoStates.MOVE_OUT_OF_STONES;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(moveOutOfStones());
                }
                break;
            case MOVE_OUT_OF_STONES://Moves robot out of the stones so that we can strafe to platform
                robot.mecanumDrive.updateFollowingDrive();
                if (elapsedTime.seconds() > 0.5) {//this wait is so that the lift has time to go down
                    robot.depositLift.grabStone();//grip the stone
                    robot.intake.setIntakePower(-0.2);//incase an extra stone got into col we outtake
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    if (foundationMoved) {//if the foundation is moved we need to path to a different loc
                        robot.mecanumDrive.follower.followTrajectory(stonesToPlatform());
                    } else {
                        robot.mecanumDrive.follower.followTrajectory(stonesToPlatform1st());
                    }
                    state = AutoStates.PATH_TO_FOUNDATION;
                }
                break;
            case PATH_TO_FOUNDATION:// this is the path to the foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.depositLift.setTargetHeight(4);//lift the lift to drop block onto platform
                    state = AutoStates.PLACE_STONE;
                    elapsedTime.reset();
                }
                break;
            case PLACE_STONE:
                //This is a stone place procedure, rn we are just dropping the stone but need to be build stacks in the future
                //In future incorporate this into driving over to foundation
                if (elapsedTime.seconds()<0.5){}
                else if (elapsedTime.seconds() < 1) {
                    robot.depositLift.setExtensionPower(1);
                } else if (elapsedTime.seconds() < 1.5) {
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 2) {
                    robot.depositLift.setExtensionPower(-1);
                } else {
                    robot.depositLift.setExtensionPower(0);
                    quarryStones.remove((Integer) currentStone);//this removes the current stone from our quarryStone array
                    robot.intake.setIntakePower(0);

                    if (foundationMoved) {//if the foundation is not moved we set the path to go
                        currentStone = quarryStones.get(0);//After both the skystones are taken we will go for the first stone in the quarry
                        robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
                        state = AutoStates.PATH_TO_STONES;
                    } else {
                        robot.mecanumDrive.platformGrab();//Grabs the foundation and waits 2 seconds for servos to move down
                        if (elapsedTime.seconds() > 2) {
                            state = AutoStates.MOVE_FOUNDATION;
                            robot.mecanumDrive.follower.followTrajectory(movePlatform());
                            foundationMoved = true;// move the stone
                        }
                    }
                }
                break;
            case MOVE_FOUNDATION://Splines to move foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    currentStone = skyPos + 3;//Get next skystone
                    robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                    robot.mecanumDrive.platformRelease();//let go of foundation

                }
                break;
            case PARK:
                robot.depositLift.setTargetHeight(0.5);
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.IDLE;

                }
                break;
            case IDLE:
                robot.mecanumDrive.stopDriveMotors();
                requestOpModeStop();
                break;
        }
        if (time > (30.0 - parkPath().duration())) {
            state = AutoStates.PARK;
            robot.mecanumDrive.follower.followTrajectory(parkPath());
        }
        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        telemetry.addData("Robot Heading", robot.mecanumDrive.getPoseEstimate().getHeading());
        robot.mecanumDrive.updatePoseEstimate();
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()));
        telemetry.update();
    }

    private Trajectory moveOutOfStones() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), (allianceColorisRed ? -40 : 40)), new SplineInterpolator(Math.toRadians(120), Math.PI))
                .build();
    }

    private Trajectory parkPath() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, (allianceColorisRed ? -36 : 36)), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                .build();
    }


    public Trajectory startToSkyStone(int skyStonePos) {
        if (0 != skyStonePos) {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 15, -35).vec(), new LinearInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), Math.PI))
                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 15, -40), new SplineInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), (allianceColorisRed ? Math.toRadians(120) : Math.toRadians(240))))
                    .build();
        }
    }

    public Trajectory stonesToPlatform1st() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, (allianceColorisRed ? -42 : 42), Math.PI))
                .splineTo(new Pose2d(48, (allianceColorisRed ? -32 : 32), Math.PI * 3 / 2), new SplineInterpolator(Math.PI, Math.toRadians((allianceColorisRed ? 270 : 90))))
                .lineTo(new Vector2d(48, -30), new ConstantInterpolator(Math.PI * 3 / 2))
                .build();
    }

    public Trajectory stonesToPlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, (allianceColorisRed ? -40 : 40), Math.PI))
                .splineTo(new Pose2d(43, (allianceColorisRed ? -45 : 45), Math.PI))
                .build();
    }

    public Trajectory movePlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(36, (allianceColorisRed ? -45 : 45), Math.PI))
                .splineTo(new Pose2d(43, (allianceColorisRed ? -45 : 45), Math.PI))
                .build();
    }

    public Trajectory platformToStones(int stone) {
        double turnAngleForCollect = 0 != stone ? Math.PI : Math.toRadians((allianceColorisRed ? -120 : 240));  //This outputs the angle that the robot should be when it goes into the quarry
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(0, (allianceColorisRed ? -36 : 36)), new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, (allianceColorisRed ? -33 : 33)), new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, quarryStonePoses[stone][1] + (allianceColorisRed ? -12 : 12)), new LinearInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), turnAngleForCollect))
                .build();

    }

    public Trajectory goIntoStones() {
        //if we drive into the side of a stone adjust the array to reflect the new position
        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        } else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
            quarryStonePoses[currentStone - 2][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        }

        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), quarryStonePoses[currentStone][1]), new ConstantInterpolator(Math.PI))
                .build();
    }

    public enum AutoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, MOVE_INTO_STONES, STONE_PICK, MOVE_OUT_OF_STONES, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
