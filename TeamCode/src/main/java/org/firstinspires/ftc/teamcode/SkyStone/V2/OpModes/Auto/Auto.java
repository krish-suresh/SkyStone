package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes.Auto;

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

    final double[][] redQuarryStonePoses = {{-27.5, -22}, {-35.5, -22}, {-43.5, -22}, {-51.5, -22}, {-59.5, -22}, {-67.5, -22}};
    //    final double[][] redQuarryStonePoses = {{-28, -22}, {-36, -22}, {-44, -22}, {-52, -22}, {-60, -22}, {-68, -22}};
    final double[][] blueQuarryStonePoses = {{-28, 22}, {-36, 22}, {-44, 22}, {-52, 22}, {-60, 22}, {-68, 22}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    ArrayList<Integer> quarryStones = new ArrayList<>();

    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private ElapsedTime endTime;
    private boolean foundationMoved = true;
    private int currentStone;
    private boolean allianceColorisRed = true;
    private double autoAddPower;
    private boolean startPark = false;
    private double waitTime = 0;
    private boolean tempUp = true;
    private boolean tempDown = true;
    private boolean waitStarted = false;

    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        camera = new Camera(this);//we should prob incorp this into the robot obj
        stickygamepad1 = new StickyGamepad(gamepad1);//for alliance sel
        elapsedTime = new ElapsedTime();
        endTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));//adds all the stones in the quarry
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
        robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED0);
//        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
        loadFromFile();
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
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, -Math.PI / 2));// Red start pos
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-36, 63, Math.PI * 3 / 2));// Blue start pos
        }
        saveToFile();
        telemetry.update();
    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
        elapsedTime.reset();
        endTime.reset();
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
            case SKYSTONE_DETECT:
                currentStone = skyPos;
                state = AutoStates.PATH_TO_STONES;
                camera.phoneCam.stopStreaming();
                robot.mecanumDrive.follower.followTrajectory(startToSkyStone(skyPos));
                break;

            case PATH_TO_STONES://This follows the path to right next to the stone,
                robot.mecanumDrive.updateFollowingDrive();

                if (foundationMoved) {
                    autoAddPower = -0.3;
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    autoAddPower = 0;
                    robot.depositLift.setTargetHeight(6);
                    state = AutoStates.STONE_PICK;
                    robot.intake.setCollectorPos(Intake.CollectorPoses.RELEASED);
                    robot.intake.setIntakePower(0.8);
                    elapsedTime.reset();

                }
                break;

            case STONE_PICK:
                robot.mecanumDrive.setMecanum(Math.toRadians(135), 0.08, 0);//This make the robot drive forward slowly
                if (robot.depositLift.isStoneInBot()) {//Once the stone is in the robot we can stop the collection and lower the lift
                    robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
                    robot.mecanumDrive.stopDriveMotors();
                    state = AutoStates.PATH_TO_FOUNDATION;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(stonesToFoundation());
                }
                break;

            case PATH_TO_FOUNDATION:// this is the path to the foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (elapsedTime.seconds() > 0.5) {
                    robot.depositLift.setTargetHeight(0);//lift the lift to drop block onto platform
                    autoAddPower = -0.3;
                }

                if (robot.mecanumDrive.getPoseEstimate().getX() > 0) {
                    robot.depositLift.grabStone();
                    robot.intake.setIntakePower(0);
                }
                if (robot.mecanumDrive.getPoseEstimate().getX() > 20) {
                    robot.depositLift.setTargetHeight(10);//lift the lift to drop block onto platform
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PLACE_STONE;
                    robot.intake.setIntakePower(0);
                    autoAddPower = 0;
                    elapsedTime.reset();
                }
                break;

            case PLACE_STONE:
                autoAddPower = 0;
                if (!foundationMoved) {
                    robot.mecanumDrive.setMecanum(0, 0.04, 0);//This make the robot drive forward slowly
                    if (elapsedTime.seconds() > 0.4) {
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);//Grabs the foundation and waits 2 seconds for servos to move down
                    }
                }
                if (elapsedTime.seconds() < 0.3) {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_TURN_1);
                } else if (elapsedTime.seconds() < 0.8) {
                    robot.mecanumDrive.stopDriveMotors();
                    robot.depositLift.releaseStone();
                } else {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED0);
                    quarryStones.remove((Integer) currentStone);//this removes the current stone from our quarryStone array
                    robot.intake.setIntakePower(0);
                    currentStone = getNextStone();
                    if (foundationMoved) {
                        //if the foundation is not moved we set the path to go
                        robot.mecanumDrive.follower.followTrajectory(foundationToStones(currentStone));
                        state = AutoStates.PATH_TO_STONES;
                        robot.depositLift.setTargetHeight(0);//lift down to under bar

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
                    robot.mecanumDrive.follower.followTrajectory(foundationToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                    robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                }
                break;
            case PARK:
                robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                robot.depositLift.setTargetHeight(0);
                autoAddPower = -0.2;
                robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED0);
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
        if (!startPark && endTime.seconds() > 30.0 - parkPath().duration()) {
            state = AutoStates.PARK;
            startPark = true;
            robot.mecanumDrive.follower.followTrajectory(parkPath());
        }
        //Dashboard Spline Drawing Start
        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.fillCircle(robot.mecanumDrive.getPoseEstimate().getX(), robot.mecanumDrive.getPoseEstimate().getY(), 3);
        Pose2d targetPose = new Pose2d(robot.mecanumDrive.getPoseEstimate().getX()+robot.mecanumDrive.follower.getLastError().getX(),robot.mecanumDrive.getPoseEstimate().getY()+robot.mecanumDrive.follower.getLastError().getY(),robot.mecanumDrive.getPoseEstimate().getHeading()+robot.mecanumDrive.follower.getLastError().getHeading());
        DashboardUtil.drawRobot(fieldOverlay, targetPose);
        packet.put("errorX",robot.mecanumDrive.follower.getLastError().getX());
        packet.put("errorY",robot.mecanumDrive.follower.getLastError().getY());
        packet.put("errorH",robot.mecanumDrive.follower.getLastError().getHeading());
        dashboard.sendTelemetryPacket(packet);
        //Dashboard Spline Drawing End
        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", robot.mecanumDrive.getPoseEstimate());
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddPower);
        telemetry.update();
    }

    private int getNextStone() {
        if (quarryStones.size() > 4) {
            return skyPos + 3;
        }
        for (int stone : quarryStones
        ) {
            if (quarryStonePoses[stone][1] == -22) {
                return stone;
            }
        }
        return quarryStones.get(quarryStones.size() - 1);
    }

    private void loadFromFile() {
//TODO IMP
    }

    private void saveToFile() {
//TODO IMP
    }

    private Trajectory parkPath() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
//                .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(),-38),new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(),Math.PI))
                .lineTo(new Vector2d(0, (allianceColorisRed ? -38 : 38)),new ConstantInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading()))
                .build();
    }


    public Trajectory startToSkyStone(int skyStonePos) {
        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        }
        else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
            quarryStonePoses[currentStone - 2][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);

        }
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(quarryStonePoses[skyStonePos][0] + 12, allianceColorisRed ? -33 : 33), new SplineInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), (allianceColorisRed ? Math.toRadians(135) : Math.toRadians(225))))
                .build();

    }

    public Trajectory stonesToFoundation() {
        if (!foundationMoved) { // Do this first
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), (allianceColorisRed ? -36 : 36)), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                    .lineTo(new Vector2d(0.0, allianceColorisRed ? -36.0 : 36.0), new ConstantInterpolator(Math.PI))
                    .lineTo(new Vector2d(48.0, allianceColorisRed ? -36.0 : 36.0), new SplineInterpolator(Math.PI, allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2))
                    .lineTo(new Vector2d(48.0, allianceColorisRed ? -24.0 : 24.0), new ConstantInterpolator(allianceColorisRed ? Math.PI * 3 / 2 : Math.PI / 2))

                    .build();
        } else {
            return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(robot.mecanumDrive.getPoseEstimate().getX(), (allianceColorisRed ? -36 : 36)), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                    .lineTo(new Vector2d(0.0, allianceColorisRed ? -36.0 : 36.0), new ConstantInterpolator(Math.PI))
                    .lineTo(new Vector2d(50, allianceColorisRed ? -42 : 42), new ConstantInterpolator(Math.PI))//TODO ALLicol
                    .build();
        }
    }

    public Trajectory moveFoundation() {
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(28, (allianceColorisRed ? -40 : 40), Math.toRadians(allianceColorisRed ? 135 : 225))) // TODO
                .reverse()
                .splineTo(new Pose2d(52, (allianceColorisRed ? -48 : 48), Math.PI))
                .build();
    }

    public Trajectory foundationToStones(int stone) {

        if (currentStone == 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
        }
        else if (currentStone > 1) {
            quarryStonePoses[currentStone - 1][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);
            quarryStonePoses[currentStone - 2][1] += (allianceColorisRed ? Robot.ROBOT_WIDTH / 2 : -Robot.ROBOT_WIDTH / 2);

        }
        return new TrajectoryBuilder(robot.mecanumDrive.getPoseEstimate(), robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(0, (allianceColorisRed ? -38 : 36)).vec(), new SplineInterpolator(robot.mecanumDrive.getPoseEstimate().getHeading(), Math.PI))
                .lineTo(new Vector2d(quarryStonePoses[stone][0] + 14, allianceColorisRed ? -36 : 36), new SplineInterpolator((allianceColorisRed ? Math.PI / 2 : Math.PI * 3 / 2), (allianceColorisRed ? Math.toRadians(135) : Math.toRadians(225))))
                .lineTo(new Vector2d(quarryStonePoses[stone][0] + 14, allianceColorisRed ? -26 : 33), new ConstantInterpolator(Math.toRadians(135)))
                .build();

    }

    public enum AutoStates {
        WAIT, SKYSTONE_DETECT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum AllianceColors {

        RED, BLUE
    }

}
