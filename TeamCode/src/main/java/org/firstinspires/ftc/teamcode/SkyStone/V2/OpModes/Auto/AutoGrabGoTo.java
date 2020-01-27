package org.firstinspires.ftc.teamcode.SkyStone.V2.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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

import org.firstinspires.ftc.teamcode.RobotLibs.StickyGamepad;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.logging.Formatter;

@Autonomous(name = "AutoGrabGoTo")
public class AutoGrabGoTo extends OpMode {
    Robot robot;
    AutoStates state = AutoStates.WAIT;
    AllianceColors allianceColor = AllianceColors.RED;
    Camera camera;
//    private FtcDashboard dashboard = FtcDashboard.getInstance();

    final double[][] redQuarryStonePoses = {{-27.5, -22}, {-35.5, -22}, {-43.5, -22}, {-51.5, -22}, {-59.5, -22}, {-67.5, -22}};
    final double[][] blueQuarryStonePoses = {{-28, 22}, {-36, 22}, {-44, 22}, {-52, 22}, {-60, 22}, {-68, 22}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    ArrayList<Integer> quarryStones = new ArrayList<>();

    private StickyGamepad stickygamepad1;
    private ElapsedTime elapsedTime;
    private int currentStone;
    private boolean allianceColorisRed = true;
    private double autoAddPower;
    private double waitTime = 0;
    private boolean tempUp = true;
    private boolean tempDown = true;
    private ElapsedTime cycleTime;
    private double lastTime = 0;
    Pose2d currentPos;
    private boolean waitStarted = false;
    private int placeHeight=8;
    private double pickY = -37;
    private Pose2d placePose = new Pose2d(36,-40,-Math.PI/2);

    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        camera = new Camera(this);//we should prob incorp this into the robot obj
        stickygamepad1 = new StickyGamepad(gamepad1);//for alliance sel
        elapsedTime = new ElapsedTime();
        cycleTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));//adds all the stones in the quarry
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
        robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED1);
        robot.depositLift.grabStone();
        loadFromFile();
    }

    @Override
    public void init_loop() {
        allianceColor = stickygamepad1.x ? AllianceColors.BLUE : AllianceColors.RED;
        telemetry.addData("ALLIANCE: ", allianceColor);
        allianceColorisRed = allianceColor == AllianceColors.RED;//This is used to assign positions for the splines based on alliance
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
//        skyPos = camera.getSkyPos(allianceColorisRed);
        skyPos = 1;
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
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, Math.PI*3 / 2));// Red start pos
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
        cycleTime.reset();
        currentStone = skyPos;
        camera.phoneCam.stopStreaming();
        robot.mecanumDrive.goToPosition(new Pose2d(-35.5,pickY,-Math.PI/2));
    }


    @Override
    public void loop() {
        robot.mecanumDrive.updatePoseEstimate();
        currentPos = robot.mecanumDrive.getPoseEstimate();
        robot.mecanumDrive.updateGoToPos();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        switch (state) {
            case WAIT:
                if (elapsedTime.seconds() > waitTime) {
                    state = AutoStates.PATH_TO_STONES;
                    elapsedTime.reset();
                    robot.depositLift.setTargetHeight(3);
                }
                break;
            case PATH_TO_STONES:
                if (currentPos.getX() > 20) {
                    autoAddPower = -0.3;
                }
                if (currentPos.getX()<10&&currentPos.getX()>0){
                    robot.mecanumDrive.goToPosition(new Pose2d(quarryStonePoses[currentStone][0], pickY,-Math.PI/2));
                }
                if (currentPos.getX()>20&&currentPos.getX()<36){
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                }
                if (currentPos.getX() < -20 && currentPos.getY() > -48) {
                    autoAddPower = 0.2;
                    robot.depositLift.setTargetHeight(4);
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO);
                    if (!waitStarted) {
                        elapsedTime.reset();
                        waitStarted = true;
                    }
                    if (elapsedTime.seconds() > 0.5) {
                        robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_DEFAULT);
                        robot.depositLift.releaseStone();
                    }
                }
                if (robot.mecanumDrive.isInRange()) {
                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_DEFAULT);
                    robot.mecanumDrive.stopDriveMotors();
                    robot.depositLift.releaseStone();
                    autoAddPower = 0;
                    state = AutoStates.STONE_PICK;
                    elapsedTime.reset();

                }
                break;

            case STONE_PICK:
                //TODO Tune this to be as fast as possible
                robot.mecanumDrive.resetControllers();
                if (elapsedTime.seconds() < 0.2) {
                    autoAddPower = -0.5;
                    robot.depositLift.setTargetHeight(0);
                } else if (elapsedTime.seconds() < 0.6) {
                    robot.depositLift.grabStone();
                } else if (elapsedTime.seconds() < 0.7) {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);
                    robot.depositLift.setTargetHeight(7);
                    autoAddPower = 0.2;
                } else {
                    state = AutoStates.PATH_TO_FOUNDATION;
                    robot.mecanumDrive.goToPosition(new Pose2d(5, -42,-Math.PI/2));
                    elapsedTime.reset();
//                    robot.mecanumDrive.goToPosition(placePose);
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);
                }
                break;
            case PATH_TO_FOUNDATION:// this is the path to the foundation
                if (currentPos.getX()>0){
                    robot.mecanumDrive.goToPosition(placePose);
                }

                if (currentPos.getX() > 20) {
                    robot.depositLift.setTargetHeight(placeHeight);//lift the lift to drop block onto platform
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_TURN_1);
                } else if (currentPos.getX() > -20) {
                    autoAddPower = 0.2;
                    robot.depositLift.setTargetHeight(1);//lift the lift to drop block onto platform
                }
                if (robot.mecanumDrive.isInRange(2,2)) {
                    state = AutoStates.PLACE_STONE;
                    robot.mecanumDrive.stopDriveMotors();
                    placeHeight += 4;
                    autoAddPower = 0;
                    elapsedTime.reset();
                }
                break;

            case PLACE_STONE:
                //TODO Make this code do stacking
                robot.mecanumDrive.resetControllers();
                if (elapsedTime.seconds() < 0.2) {
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 0.3) {
//                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_DEFAULT);
                } else {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);
                    quarryStones.remove((Integer) currentStone);//this removes the current stone from our quarryStone array
                    currentStone = getNextStone();
                    robot.mecanumDrive.goToPosition(new Pose2d(0, -42,-Math.PI/2));
                    state = AutoStates.PATH_TO_STONES;
                    if (false&&quarryStones.size() == 2) {
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);//Grabs the foundation and waits 2 seconds for servos to move down
                        state = AutoStates.MOVE_FOUNDATION;
                        robot.mecanumDrive.follower.followTrajectory(moveFoundation());
                    }
                }
                break;
            case MOVE_FOUNDATION://Splines to move foundation
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PARK;
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                    robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                    robot.mecanumDrive.follower.followTrajectory(parkPath());
                }
                break;
            case PARK:
                robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                robot.depositLift.setTargetHeight(0);
                autoAddPower = -0.2;

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

        //Dashboard Spline Drawing Start
        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.fillCircle(currentPos.getX(), currentPos.getY(), 3);
//        dashboard.sendTelemetryPacket(packet);
        //Dashboard Spline Drawing End
        robot.telemetry.addData("STATE", state);
        robot.telemetry.addData("Robot Pos", currentPos);
        robot.telemetry.addData("errorX",robot.mecanumDrive.PID_FORWARD.getLastError());
        robot.telemetry.addData("errorY",robot.mecanumDrive.PID_STRAFE.getLastError());
        robot.telemetry.addData("errorH",robot.mecanumDrive.PID_HEADING.getLastError());
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddPower);
        robot.telemetry.update();
    }

    private int getNextStone() {
        if (quarryStones.size() > 4) {
            return skyPos + 3;
        }
        return quarryStones.get(0);
    }

    private void loadFromFile() {
//TODO IMP
    }

    private void saveToFile() {
//TODO IMP
    }

    private Trajectory parkPath() {
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(currentPos.getX(), -38), new SplineInterpolator(currentPos.getHeading(), Math.PI))
                .lineTo(new Vector2d(0, (allianceColorisRed ? -38 : 38)), new ConstantInterpolator(currentPos.getHeading()))
                .build();
    }


    public Trajectory moveFoundation() {
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(28, (allianceColorisRed ? -40 : 40), Math.toRadians(allianceColorisRed ? 135 : 225))) // TODO
                .reverse()
                .splineTo(new Pose2d(52, (allianceColorisRed ? -48 : 48), Math.PI))
                .build();
    }

    public enum AutoStates {
        WAIT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_STONE, MOVE_FOUNDATION, PARK, IDLE
    }

    public enum AllianceColors {
        RED, BLUE
    }

}
