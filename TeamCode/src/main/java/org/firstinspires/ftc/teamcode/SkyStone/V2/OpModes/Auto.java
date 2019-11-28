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

//TODO Retune the turning and make movement 30ips
//TODO localization is iffy right now, need to make 3 tracker wheel so no imu dependency
//TODO Bulk reads
//TODO TEST teleop to auto switcher https://www.kno3.net/autonomous-teleop-transitioner
//TODO Check alliance partner collisions
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

    final double[][] redQuarryStonePoses = {{-28, -22}, {-36, -22}, {-44, -22}, {-52, -12}, {-60, -12}, {-68, -12}};
    final double[][] blueQuarryStonePoses = {{-28, 22}, {-36, 22}, {-44, 22}, {-52, 12}, {-60, 12}, {-68, 12}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    ArrayList<Integer> quarryStones = new ArrayList<>();

    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private boolean foundationMoved = false;
    private int currentStone;
    private boolean allianceColorisRed = true;
    private double autoAddPower;
    private boolean startPark = false;

    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        camera = new Camera(this);//we should prob incorp this into the robot obj
        stickygamepad1 = new StickyGamepad(gamepad1);//for alliance sel
        elapsedTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));//adds all the stones in the quarry
        robot.mecanumDrive.platformRelease();
        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
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
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, 63, Math.PI * 3 / 2));// Blue start pos
        }

    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
        allianceColorisRed = allianceColor == AllianceColors.RED;//This is used to assign positions for the splines based on alliance
        if (!allianceColorisRed) {
            skyPos = (skyPos == 0 ? 2 : (skyPos == 2) ? 0 : 1);
        }
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
                if (foundationMoved) {
                    autoAddPower = -0.3;
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    autoAddPower = 0;
                    robot.depositLift.setTargetHeight(6);
                    state = AutoStates.STONE_PICK;
                    robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                    robot.intake.setIntakePower(0.8);//Since move into stones is skipped, we still need to update the intake stuff

                }
                break;

            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.PI, 0.15, 0);//This make the robot drive forward slowly
                //TODO add smart collection
                if (robot.depositLift.isStoneInBot()) {//Once the stone is in the robot we can stop the collection and lower the lift
                    robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
                    robot.intake.setIntakePower(-0.3);//Since move into stones is skipped, we still need to update the intake stuff
                    robot.mecanumDrive.stopDriveMotors();//stops DT
                    state = AutoStates.PATH_TO_FOUNDATION;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(stonesToPlatform());
                }
                break;

            case PATH_TO_FOUNDATION:// this is the path to the foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (robot.mecanumDrive.getPoseEstimate().getX() > 0) {
                    robot.depositLift.grabStone();
                }
                if (robot.mecanumDrive.getPoseEstimate().getX() > 20) {
                    robot.depositLift.setTargetHeight(8);//lift the lift to drop block onto platform
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PLACE_STONE;
                    robot.intake.setIntakePower(0);
                    elapsedTime.reset();
                }
                break;

            case PLACE_STONE:
                autoAddPower = 0;
                //This is a stone place procedure, rn we are just dropping the stone but need to be build stacks in the future
                //In future incorporate this into driving over to foundation
                if (!foundationMoved) {
                    robot.mecanumDrive.setMecanum(0, 0.04, 0);//This make the robot drive forward slowly
                }

                if (elapsedTime.seconds() < 0.5) {
//                    robot.depositLift.setExtensionPower(1);
                } else if (elapsedTime.seconds() < 1) {
                    if (!foundationMoved) {
                        robot.mecanumDrive.platformGrab();//Grabs the foundation and waits 2 seconds for servos to move down
                    }
                } else if (elapsedTime.seconds() < 1.25) {
                    robot.mecanumDrive.stopDriveMotors();
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 2) {
//                    robot.depositLift.setExtensionPower(-1);
                } else {
//                    robot.depositLift.setExtensionPower(0);
                    quarryStones.remove((Integer) currentStone);//this removes the current stone from our quarryStone array
                    robot.intake.setIntakePower(0);

                    if (foundationMoved) {
                        //if the foundation is not moved we set the path to go
//                        currentStone = quarryStones.get(0);//After both the skystones are taken we will go for the first stone in the quarry
//                        robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
//                        state = AutoStates.PATH_TO_STONES;
                        startPark = true;
                    } else {
                        state = AutoStates.MOVE_FOUNDATION;
                        robot.mecanumDrive.stopDriveMotors();
                        robot.mecanumDrive.follower.followTrajectory(movePlatform());
                        foundationMoved = true;// move the stone

                    }
                }
                break;

            case MOVE_FOUNDATION://Splines to move foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
//                    if (elapsedTime.seconds()<1){
//                        robot.mecanumDrive.setMecanum(0, -0.5, 0);//This make the robot drive forward slowly
//
//                    }
                    robot.mecanumDrive.setPoseEstimate(new Pose2d(45, allianceColorisRed ? -44 : 44, Math.PI));
                    currentStone = skyPos + 3;//Get next skystone
                    robot.mecanumDrive.follower.followTrajectory(platformToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                    robot.mecanumDrive.platformRelease();//let go of foundation

                }
                break;

            case PARK:
                robot.depositLift.setTargetHeight(0);
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
        if (!startPark && time > parkPath().duration()) {
            state = AutoStates.PARK;
            startPark = true;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        telemetry.addData("Robot Heading", robot.mecanumDrive.getPoseEstimate().getHeading());
        robot.mecanumDrive.updatePoseEstimate();
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddPower);
        telemetry.update();
    }


    private Trajectory parkPath() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(0, (allianceColorisRed ? -36 : 36)))
                .build();
    }


    public Trajectory startToSkyStone(int skyStonePos) {

        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 18, allianceColorisRed ? -35 : 35).vec(), new LinearInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), (allianceColorisRed ? Math.PI / 2 : -Math.PI / 2)))
                .build();
    }

    public Trajectory stonesToPlatform() {
        if (foundationMoved) {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .reverse()
                    .splineTo(new Pose2d(0, (allianceColorisRed ? -42 : 42), Math.PI))
                    .splineTo(new Pose2d(48, (allianceColorisRed ? -32 : 32), (allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2)))
                    .splineTo(new Pose2d(48, (allianceColorisRed ? -28 : 28), (allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2)))
                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .reverse()
                    .splineTo(new Pose2d(0, (allianceColorisRed ? -36 : 36), Math.PI))
                    .splineTo(new Pose2d(43, (allianceColorisRed ? -40 : 40), Math.PI))
                    .build();
        }
    }

    public Trajectory movePlatform() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(foundationMoved ? 20 : 30, (allianceColorisRed ? -50 : 50), Math.PI))
                .reverse()
                .splineTo(new Pose2d(48, (allianceColorisRed ? -48 : 48), Math.PI))
                .build();
    }

    public Trajectory platformToStones(int stone) {

        //if we drive into the side of a stone adjust the array to reflect the new position
        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        } else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
            quarryStonePoses[currentStone - 2][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        }

        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .strafeRight((allianceColorisRed ? 6 : -6))
                .lineTo(new Pose2d(0, (allianceColorisRed ? -40 : 40)).vec(), new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, (allianceColorisRed ? -32 : 32), Math.PI),new SplineInterpolator(Math.PI,Math.toRadians(allianceColorisRed?120:240)))
                .build();

    }

    public enum AutoStates {
        SKYSTONE_DETECT, PATH_TO_STONES, MOVE_INTO_STONES, STONE_PICK, MOVE_OUT_OF_STONES, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
