package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotLibs.DashboardUtil;
import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Auto")
public class Auto extends OpMode {
    Robot robot;
    AutoStates state = AutoStates.WAIT;
    AllianceColors allianceColor = AllianceColors.RED;
    Camera camera;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
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
    final double[][] blueQuarryStonePoses = {{-28, 22}, {-36, 22}, {-44, 22}, {-52, 22}, {-60, 22}, {-68, 22}};

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
    private double waitTime = 0;
    private boolean tempUp = true;
    private boolean tempDown = true;

    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        camera = new Camera(this);//we should prob incorp this into the robot obj
        stickygamepad1 = new StickyGamepad(gamepad1);//for alliance sel
        elapsedTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));//adds all the stones in the quarry
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
//        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
    }

    @Override
    public void init_loop() {
        allianceColor = stickygamepad1.x ? AllianceColors.BLUE : AllianceColors.RED;
        foundationMoved = stickygamepad1.y;//if we click y then it will skip the foundation moving state
        telemetry.addData("ALLIANCE: ", allianceColor);
        telemetry.addData("MOVE FOUNDATION", !foundationMoved);
        allianceColorisRed = allianceColor == AllianceColors.RED;//This is used to assign positions for the splines based on alliance
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
        skyPos = camera.getSkyPos(allianceColorisRed);
        telemetry.addData("SkyPos: ", skyPos);
        if (stickygamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            waitTime = Range.clip(waitTime + 0.5, 0, 5);
        } else if (stickygamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            waitTime = Range.clip(waitTime - 0.5, 0, 5);
        }
        telemetry.addData("WAIT: ", waitTime);
        stickygamepad1.update();
        if (allianceColor == AllianceColors.RED) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, -63, Math.PI / 2));// Red start pos
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, 63, Math.PI * 3 / 2));// Blue start pos
        }
        telemetry.update();
    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
        elapsedTime.reset();
    }


    @Override
    public void loop() {
        robot.mecanumDrive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        switch (state) {
            case WAIT:
                if (elapsedTime.seconds() > waitTime) {
                    state = AutoStates.SKYSTONE_DETECT;
                    elapsedTime.reset();
                }
                break;
            case SKYSTONE_DETECT://This is the starting state where we create the traj to the correct stone and lift the height a bit
                robot.mecanumDrive.follower.followTrajectory(startToSkyStone(skyPos));
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, startToSkyStone(skyPos).getPath());
                currentStone = skyPos;
                state = AutoStates.PATH_TO_STONES;
                camera.phoneCam.stopStreaming();
                break;

            case PATH_TO_STONES://This follows the path to right next to the stone,
                robot.mecanumDrive.updateFollowingDrive();

                Trajectory trajectory = robot.mecanumDrive.follower.getTrajectory();
                fieldOverlay.setStroke("#F44336");
                double t = robot.mecanumDrive.follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                if (foundationMoved) {
                    autoAddPower = -0.3;
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    autoAddPower = 0;
                    robot.depositLift.setTargetHeight(6);
                    state = AutoStates.STONE_PICK;
                    robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    robot.intake.setIntakePower(1);
                    elapsedTime.reset();
                }
                break;

            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.PI, 0.15, 0);//This make the robot drive forward slowly
                if (elapsedTime.seconds() > 1.2) {
                    //TODO backup and turn a bit
                }
                //TODO add if block in collector for more than a sec then try opening and closing col or rev one side
                if (robot.depositLift.isStoneInBot()) {//Once the stone is in the robot we can stop the collection and lower the lift
                    robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
                    robot.intake.setIntakePower(-0.3);//Since move into stones is skipped, we still need to update the intake stuff
                    robot.mecanumDrive.stopDriveMotors();//stops DT
                    state = AutoStates.PATH_TO_FOUNDATION;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(stonesToFoundation());
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
                //TODO Maybe take this out
                if (robot.depositLift.getAbsLiftHeight() > 3) {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_TURN_1);
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PLACE_STONE;
                    robot.intake.setIntakePower(0);
                    elapsedTime.reset();
                }
                break;

            case PLACE_STONE:
                autoAddPower = 0;
                if (!foundationMoved) {
                    robot.mecanumDrive.setMecanum(0, 0.04, 0);//This make the robot drive forward slowly
                    if (elapsedTime.seconds() > 0.5) {
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);//Grabs the foundation and waits 2 seconds for servos to move down
                    }
                }
                if (elapsedTime.seconds() < 0.3) {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_TURN_1);
                } else if (elapsedTime.seconds() < 0.75) {
                    robot.mecanumDrive.stopDriveMotors();
                    robot.depositLift.releaseStone();
                } else {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED);
                    quarryStones.remove((Integer) currentStone);//this removes the current stone from our quarryStone array
                    robot.intake.setIntakePower(0);

                    if (foundationMoved) {
                        //if the foundation is not moved we set the path to go
                        currentStone = quarryStones.get(0);//After both the skystones are taken we will go for the first stone in the quarry
                        robot.mecanumDrive.follower.followTrajectory(foundationToStones(currentStone));
                        state = AutoStates.PATH_TO_STONES;
                        startPark = true;
                    } else {
                        state = AutoStates.MOVE_FOUNDATION;
                        robot.mecanumDrive.stopDriveMotors();
                        robot.mecanumDrive.follower.followTrajectory(moveFoundation());
                        foundationMoved = true;// move the stone

                    }
                }
                break;

            case MOVE_FOUNDATION://Splines to move foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    currentStone = skyPos + 3;//Get next skystone
                    robot.mecanumDrive.follower.followTrajectory(foundationToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                    robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                }
                break;
            case PARK:
                robot.depositLift.setTargetHeight(0);
                autoAddPower = (robot.depositLift.getAbsLiftHeight() > 0.5) ? -0.2 : 0;
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
        if (!startPark && 30f < time + parkPath().duration()) {
            state = AutoStates.PARK;
            startPark = true;
            robot.mecanumDrive.follower.followTrajectory(parkPath());
        }
        //Dashboard Spline Drawing Start

        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.fillCircle(robot.mecanumDrive.getPoseEstimate().getX(), robot.mecanumDrive.getPoseEstimate().getY(), 3);
        dashboard.sendTelemetryPacket(packet);

        //Dashboard Spline Drawing End
        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddPower);
        telemetry.update();
    }


    private Trajectory startTraj() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(robot.mecanumDrive.getPoseEstimate())
                .build();
    }

    private Trajectory parkPath() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(0, (allianceColorisRed ? -36 : 36)))
                .build();
    }


    public Trajectory startToSkyStone(int skyStonePos) {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(quarryStonePoses[skyStonePos][0] + 10, allianceColorisRed ? -35 : 35).vec(), new SplineInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), (allianceColorisRed ? Math.toRadians(120) : -Math.PI / 2)))
                .build();
    }

    public Trajectory stonesToFoundation() {
        if (!foundationMoved) {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), (allianceColorisRed ? -36 : 36)), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                    .reverse()
                    .splineTo(new Pose2d(0, (allianceColorisRed ? -42 : 42), Math.PI))
                    .splineTo(new Pose2d(48, (allianceColorisRed ? -32 : 32)), new SplineInterpolator(Math.PI, allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2))
                    .splineTo(new Pose2d(48, (allianceColorisRed ? -28 : 28), (allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2)))
                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), (allianceColorisRed ? -36 : 36)), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                    .reverse()
                    .splineTo(new Pose2d(0, (allianceColorisRed ? -36 : 36), Math.PI))
                    .splineTo(new Pose2d(43, (allianceColorisRed ? -40 : 40), Math.PI))
                    .build();
        }
    }

    public Trajectory moveFoundation() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(28, (allianceColorisRed ? -48 : 48), Math.PI))
                .reverse()
                .splineTo(new Pose2d(48, (allianceColorisRed ? -48 : 48), Math.PI))
                .build();
    }

    public Trajectory foundationToStones(int stone) {

        //if we drive into the side of a stone adjust the array to reflect the new position
        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        } else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
//            quarryStonePoses[currentStone - 2][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        }

        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(0, (allianceColorisRed ? -36 : 36)).vec(), new ConstantInterpolator(Math.PI))
                .lineTo(new Pose2d(quarryStonePoses[stone][0] + 15, (allianceColorisRed ? -36 : 36)).vec(), new SplineInterpolator(Math.PI, Math.toRadians(allianceColorisRed ? 120 : 240)))
                .splineTo(new Pose2d(quarryStonePoses[stone][0] + 15, quarryStonePoses[stone][1] + (allianceColorisRed ? -15 : 15)), new ConstantInterpolator(Math.toRadians(allianceColorisRed ? 120 : 240)))
                .build();

    }

    public enum AutoStates {
        WAIT, SKYSTONE_DETECT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
