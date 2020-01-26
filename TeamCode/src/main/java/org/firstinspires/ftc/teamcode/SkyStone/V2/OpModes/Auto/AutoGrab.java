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
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

/*

Proposed path:
-1st stone to foundation
-grab foundation
-pull foundation into center bridge linearly (no turn yet)
-get blocks 2, 3, 4 normally
-get block 5 and grab foundation
-slide right and then quarter-circle to corner
    -park with tape measurer during turn at the end

 */


/*

Things to add:
-solution to grab stone against the wall with deposit rotation
-tape measurer for parking in auto/tele

 */

@Autonomous(name = "AutoGrab")
public class AutoGrab extends OpMode {
    Robot robot;
    AutoStates state = AutoStates.WAIT;
    AllianceColors allianceColor = AllianceColors.RED;
    Camera camera;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    final double[][] redQuarryStonePoses = {{-27.5, -22}, {-35.5, -22}, {-43.5, -22}, {-51.5, -22}, {-59.5, -22}, {-67.5, -22}};

    //    final double[][] redQuarryStonePoses = {{-27.5, -22}, {-35.5, -22}, {-43.5, -22}, {-51.5, -22}, {-59.5, -22}, {-67.5, -22}};
    //    final double[][] redQuarryStonePoses = {{-28, -22}, {-36, -22}, {-44, -22}, {-52, -22}, {-60, -22}, {-68, -22}};
    final double[][] blueQuarryStonePoses = {{-28, 22}, {-36, 22}, {-44, 22}, {-52, 22}, {-60, 22}, {-68, 22}};

    double[][] quarryStonePoses;
    private int skyPos = 0;
    ArrayList<Integer> quarryStones = new ArrayList<>();

    private boolean tempUp = true;
    private boolean tempDown = true;

    private ElapsedTime elapsedTime;
    private int currentStone;
    private boolean allianceColorisRed = true;

    private double waitTime = 0;

    private double autoAddLiftPower;
    private double pickY = -36.5;
    private double pickXAdd = 0;
    private double placeX = 40;
    private int placeHeight = 8;

    private ElapsedTime cycleTime;
    private double lastTime = 0;
    private long cycleTimeLast = 0;

    Pose2d currentPos;

    private boolean waitStarted = false;

    public double foundationPushDistance = 25.0;


    @Override
    public void init() {
        robot = new Robot(this);//Makes robot obj
        camera = new Camera(this);//we should prob incorp this into the robot obj
        elapsedTime = new ElapsedTime();
        cycleTime = new ElapsedTime();
        robot.intake.setCollectorPos(Intake.CollectorPoses.FOLDED_IN);
        quarryStones.addAll(Arrays.asList(0, 1, 2, 3, 4, 5));//adds all the stones in the quarry
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
        robot.depositLift.setExtend(DepositLift.ExtendStates.RETRACTED1);
//        AutoTransitioner.transitionOnStop(this, "Tele");//transition from auto to tele when auto ends
        robot.depositLift.grabStone();
        loadFromFile();
    }

    @Override
    public void init_loop() {
        allianceColor = robot.stickyGamepad1.x ? AllianceColors.BLUE : AllianceColors.RED;
        telemetry.addData("ALLIANCE: ", allianceColor);
        allianceColorisRed = allianceColor == AllianceColors.RED;//This is used to assign positions for the splines based on alliance
        quarryStonePoses = (allianceColor == AllianceColors.RED) ? redQuarryStonePoses : blueQuarryStonePoses;
//        skyPos = camera.getSkyPos(allianceColorisRed);
        skyPos = 1;
        telemetry.addData("SkyPos: ", skyPos);
        if (robot.stickyGamepad1.dpad_up == tempUp) {
            tempUp = !tempUp;
            waitTime = Range.clip(waitTime + 0.5, 0, 5);
        } else if (robot.stickyGamepad1.dpad_down == tempDown) {
            tempDown = !tempDown;
            waitTime = Range.clip(waitTime - 0.5, 0, 5);
        }
        telemetry.addData("WAIT: ", waitTime);
        robot.stickyGamepad1.update();
        if (allianceColor == AllianceColors.RED) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, Math.PI * 3 / 2));// Red start pos
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
    }

    // Outline for Auto
        // WAIT
        // START_TO_SKYSTONE
        // STONE PICK
        // STONES TO FOUNDATION
        // 1FOUNDATION TO STONES WITH PLATFORM MOVE
        // STONE PICK
        // STONES TO FOUNDATION
        // 2STONE PLACE
        // FOUNDATION TO STONES
        // STONE PICK
        // STONES TO FOUNDATION
        // 3STONE PLACE
        // FOUNDATION TO STONES
        // STONE PICK
        // STONES TO FOUNDATION
        // 4STONE PLACE
        // FOUNDATION TO STONES
        // STONE PICK
        // STONES TO FOUNDATION
        // 5STONE PLACE
        // FOUNDATION TO STONES
        // STONE PICK
        // STONES TO FOUNDATION
        // 6STONE PLACE
        // MOVE FOUNDATION/PARK


    @Override
    public void loop() {
        robot.mecanumDrive.updatePoseEstimate();
        currentPos = robot.mecanumDrive.getPoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        switch (state) {

            // delay the robot from starting until a specified time
            case WAIT:
                if (elapsedTime.seconds() > waitTime) {
                    state = AutoStates.PATH_TO_STONES;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(startToSkyStone(skyPos));
                    robot.depositLift.setTargetHeight(4);
                }
                break;

            // from the wall to the first stone
            case PATH_TO_STONES:
                robot.mecanumDrive.updateFollowingDrive();
                if (currentPos.getX() > 20) {
                    autoAddLiftPower = -0.3;
                }
                if (currentPos.getX() > 20 && currentPos.getX() < 36) {
                    robot.depositLift.setTargetHeight(0);//lift down to under bar
                }
                if (currentPos.getX() < -20 && currentPos.getY() > -60) {
                    autoAddLiftPower = 0.2;
                    robot.depositLift.setTargetHeight(5);
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO);
                    if (!waitStarted) {
                        elapsedTime.reset();
                        waitStarted = true;
                    }
                    if (elapsedTime.seconds() > 0.3) {
                        robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_ROTATED);
                        robot.depositLift.releaseStone();
                    }
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    robot.mecanumDrive.goToPosition(new Pose2d(quarryStonePoses[currentStone][0] + pickXAdd, allianceColorisRed ? pickY : 36, -Math.PI / 2));
                    if (robot.mecanumDrive.isInRange()) {
                        robot.mecanumDrive.stopDriveMotors();
                        robot.depositLift.releaseStone();
                        autoAddLiftPower = 0;
                        state = AutoStates.STONE_PICK;
                        elapsedTime.reset();
                    } else {
                        state = AutoStates.ZERO_POSITION;
                    }
                }
                break;

            // use PID to get within range after most of movement controlled by follower
            case ZERO_POSITION:
                robot.mecanumDrive.updateGoToPos();
                if (robot.mecanumDrive.isInRange()) {
                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_ROTATED);
                    robot.mecanumDrive.stopDriveMotors();
                    robot.depositLift.releaseStone();
                    autoAddLiftPower = 0;
                    state = AutoStates.STONE_PICK;
                    elapsedTime.reset();
                }
                break;

            // pick up stone from line
            case STONE_PICK:
                if (elapsedTime.seconds() < 0.1) {      // bring lift down
                    autoAddLiftPower = -0.5;
                    robot.depositLift.setTargetHeight(0);
                } else if (elapsedTime.seconds() < 0.4) {       // grab stone
                    robot.depositLift.grabStone();
                } else if (elapsedTime.seconds() < 0.5) {       // bring lift up and extend out
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);
                    robot.depositLift.setTargetHeight(7);
                    autoAddLiftPower = 0.2;
                } else {            // path towards the foundation
                    state = AutoStates.PATH_TO_FOUNDATION;
                    elapsedTime.reset();
                    robot.mecanumDrive.follower.followTrajectory(stonesToFoundation());
                }
                break;

            // path from stones to foundation
            case PATH_TO_FOUNDATION:
                robot.mecanumDrive.updateFollowingDrive();
                if (currentPos.getX() > 20) {
                    robot.depositLift.setTargetHeight(placeHeight);    // lift the lift to drop block onto platform
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_TURN_1);        // bring lift out
                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_STRAIGHT);       // rotate block
                } else if (currentPos.getX() > -20) {
                    autoAddLiftPower = 0.2;
                    robot.depositLift.setTargetHeight(1);    // lift the lift to drop block onto platform
                }
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PLACE_STONE;
                    placeHeight += 4;
                    robot.intake.setIntakePower(0);
                    autoAddLiftPower = 0;
                    elapsedTime.reset();
                    robot.mecanumDrive.stopDriveMotors();
                }
                break;

            case FIRST_FOUNDATION_TO_STONE:
                robot.mecanumDrive.updateFollowingDrive();
                robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);
                if (elapsedTime.seconds() < 0.1) {   // bring lift down to proper height
                    autoAddLiftPower = -0.5;
                    robot.depositLift.setTargetHeight(1.5);
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 0.2) {   // pick up lift, unrotate and bring
                    autoAddLiftPower = 0.3;
                    robot.depositLift.setTargetHeight(3);
                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_ROTATED);
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);

                }


                // simultaneously grab foundation
                // path towards bridge a few inches
                // release the foundation grab
                // path to stones to get next stone
                break;


            // place stone onto foundation
            case PLACE_STONE:
                //TODO Make this code do stacking
                if (elapsedTime.seconds() < 0.1) {          // open the grab
                    robot.depositLift.releaseStone();
                } else if (elapsedTime.seconds() < 0.2) {       // make grabber not rotated
                    robot.depositLift.rotation.setPosition(robot.depositLift.ROTATION_ROTATED);
                } else {
                    robot.depositLift.setExtend(DepositLift.ExtendStates.EXTEND_AUTO_2);    // bring grabber close to the robot
                    quarryStones.remove((Integer) currentStone);    // this removes the current stone from our quarryStone array
                    currentStone = getNextStone();
                    robot.mecanumDrive.follower.followTrajectory(foundationToStones(currentStone));
                    state = AutoStates.PATH_TO_STONES;
                    if (false && quarryStones.size() == 2) {    // grabbing foundation currently turned off
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);    // Grabs the foundation and waits 2 seconds for servos to move down
                        state = AutoStates.MOVE_FOUNDATION;
                        robot.mecanumDrive.stopDriveMotors();
                        robot.mecanumDrive.follower.followTrajectory(moveFoundation());
                    }
                }
                break;


            case PLACE_LAST_STONE:
                break;


            // splines to move foundation
            case MOVE_FOUNDATION:
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.PARK;
                    robot.depositLift.setTargetHeight(0);    // lift down to under bar
                    robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                    robot.mecanumDrive.follower.followTrajectory(parkPath());
                }
                break;

            // go from current position to parked (close to bridge)
            case PARK:
                robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);    // ensure we aren't grabbing the foundation when we go park
                robot.depositLift.setTargetHeight(0);
                autoAddLiftPower = -0.2;

                autoAddLiftPower = (robot.depositLift.getAbsLiftHeight() > 0.5) ? -0.2 : 0;             // slow down bringing down the lift if its below 0.5
                robot.mecanumDrive.updateFollowingDrive();
                if (!robot.mecanumDrive.follower.isFollowing()) {
                    state = AutoStates.IDLE;

                }
                break;

            // use to stop early or waste time during auto
            case IDLE:
                robot.mecanumDrive.stopDriveMotors();
                robot.opModeIsActive = false;
                requestOpModeStop();
                break;
        }

        //Dashboard Spline Drawing Start
        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.fillCircle(currentPos.getX(), currentPos.getY(), 3);
        DashboardUtil.drawRobot(fieldOverlay, new Pose2d(currentPos.getX() - robot.mecanumDrive.follower.getLastError().getX(), currentPos.getY() - robot.mecanumDrive.follower.getLastError().getY(), currentPos.getHeading() + robot.mecanumDrive.follower.getLastError().getHeading()));
        packet.put("errorX", robot.mecanumDrive.follower.getLastError().getX());
        packet.put("errorY", robot.mecanumDrive.follower.getLastError().getY());
        packet.put("errorH", Math.toDegrees(robot.mecanumDrive.follower.getLastError().getHeading()));
        packet.put("Ref Rate", 1.0 / ((cycleTime.nanoseconds() - cycleTimeLast) / 1000000000.0));
        cycleTimeLast = cycleTime.nanoseconds();
        dashboard.sendTelemetryPacket(packet);
        //Dashboard Spline Drawing End

        telemetry.addData("STATE", state);
        telemetry.addData("Robot Pos", currentPos);
        robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddLiftPower);
        telemetry.update();
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


    public Trajectory startToSkyStone(int skyStonePos) {
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
//                .lineTo(new Vector2d(currentPos.getX(), -46), new ConstantInterpolator(Math.PI * 3 / 2))
                .lineTo(new Vector2d(quarryStonePoses[skyStonePos][0], allianceColorisRed ? pickY : 33), new ConstantInterpolator(-Math.PI / 2))
                .build();

    }

    public Trajectory stonesToFoundation() {

//        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
//                .lineTo(new Pose2d(-18, (allianceColorisRed ? -36 : 40)).vec(), new SplineInterpolator(Math.toRadians(-90),Math.toRadians(-179.9)))
//                .lineTo(new Pose2d(24, (allianceColorisRed ? -36 : 40)).vec(), new ConstantInterpolator(Math.toRadians(-179.9)))
//                .build();
//        placeX+=4;
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(-12, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(-90)))
                .lineTo(new Pose2d(0, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(-90)))
                .lineTo(new Pose2d(12, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(-90)))
                .lineTo(new Vector2d(placeX, allianceColorisRed ? -31 : 38), new ConstantInterpolator(-Math.PI / 2))//TODO ALLicol
                .build();
    }


    public Trajectory moveFoundation() {
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .splineTo(new Pose2d(28, (allianceColorisRed ? -40 : 40), Math.toRadians(allianceColorisRed ? 135 : 225))) // TODO
                .reverse()
                .splineTo(new Pose2d(52, (allianceColorisRed ? -48 : 48), Math.PI))
                .build();
    }

    public Trajectory foundationToStonesWithPlatform(int stone) {
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .lineTo(new Vector2d(placeX - foundationPushDistance, allianceColorisRed ? -44 : 44), new ConstantInterpolator(allianceColorisRed ? Math.PI / 2 : -Math.PI / 2))
                .addMarker(new Function0<Unit>() {
                    public Unit invoke() {
                        //release the platform once we reach where it should be
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
                        return Unit.INSTANCE;
                    }
                })
                .lineTo(new Pose2d(0, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(270)))
                .lineTo(new Pose2d(-12, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(270)))
                .lineTo(new Vector2d(quarryStonePoses[stone][0] + pickXAdd, allianceColorisRed ? pickY : 36), new ConstantInterpolator(Math.toRadians(270)))
                .build();
    }

    public Trajectory foundationToStones(int stone) {

//        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
//                .lineTo(new Pose2d(-18, (allianceColorisRed ? -36 : 40)).vec(), new ConstantInterpolator(Math.toRadians(-179.9)))
//                .lineTo(new Vector2d(quarryStonePoses[stone][0], allianceColorisRed ? pickY : 36), new SplineInterpolator(Math.toRadians(-179.9),Math.toRadians(-90)))
//                .build();
        pickY -= 0.1;
        pickXAdd += 0.75;
        return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                .lineTo(new Pose2d(12, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(270)))
                .lineTo(new Pose2d(0, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(270)))
                .lineTo(new Pose2d(-12, (allianceColorisRed ? -50 : 40)).vec(), new ConstantInterpolator(Math.toRadians(270)))
                .lineTo(new Vector2d(quarryStonePoses[stone][0] + pickXAdd, allianceColorisRed ? pickY : 36), new ConstantInterpolator(Math.toRadians(270)))
                .build();

    }

    public enum AutoStates {
        WAIT, PATH_TO_STONES, STONE_PICK, PATH_TO_FOUNDATION, PLACE_FIRST_STONE, PLACE_STONE, PLACE_LAST_STONE, MOVE_FOUNDATION, PARK, IDLE, ZERO_POSITION, FIRST_FOUNDATION_TO_STONE;
    }

    public enum AllianceColors {
        RED, BLUE
    }

}
