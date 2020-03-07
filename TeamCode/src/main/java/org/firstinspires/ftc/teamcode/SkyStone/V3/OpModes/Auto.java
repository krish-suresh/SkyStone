//V3

package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab.GRAB_DIFF_TIME;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab.GRAB_TIME;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab.PLACE_DIFF_TIME;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab.PLACE_TIME;
import static org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab.GrabState.OPEN;


import java.util.ArrayList;

import kotlin.Unit;

    /*
      Blue    |   Foundation side red
      (+,+)   |   (+,-)
              |
    --------------------
              |
      (-,+)   |   (-,-)
      Blue    |   Quarry side red
     */

@Autonomous(name = "Auto")
public class Auto extends OpMode {

    // Start object/value creation
    Robot robot;
    Camera camera;

    boolean tempUp = true;      // used for waitTime setting
    boolean tempDown = true;
    boolean tempLeft = true;    // used for stonesToPlace setting
    boolean tempRight = true;

    int skystone;       // from Camera, which position (0, 1, 2) the skystone is
    int stonesPlaced = 0;
    ArrayList<Integer> stones = new ArrayList<>(6);

    boolean allianceColorIsRed = true;

    double waitTime = 0;

    AutoState oopState;

    static final double UP = 0;
    static final double DOWN = Math.PI;
    static final double LEFT = Math.PI / 2;
    static final double RIGHT = -Math.PI / 2;
    static double OUT;
    private double HEADING;

    Pose2d currentPos;

    final double[][] redQuarryStonePoses = {{-27, -22}, {-35, -22}, {-43, -22}, {-51, -22}, {-59, -22}, {-67, -22}};
    final double[][] blueQuarryStonePoses = {{-23.5, 22}, {-31.5, 22}, {-39.5, 22}, {-47.5, 22}, {-55.5, 22}, {-61.5, 22}};
    double[][] quarryStonePoses;

    ElapsedTime time;

    private double pickY = -35;                         // Y-distance at which we pick stones
    private double FINAL_PICK_Y;
    private double placeX = 56;                         // X-distance where we place stones on the foundation
    private double pickXAdd = 0;                        // Additional X pos of picking the stone (used for tuning)

    private double BRIDGE_DISTANCE = 39;                // Y-distance at which we go around the bridge
    private double FINAL_BRIDGE_DISTANCE;
    private int currentStone;
    private final double TURN_GRAB_ADJUST = 5.25;          // X-distance from the center of the stone at which we will pick up the stones due to turning grab
    private int stonesToPlace = 4;
    // End object/value creation


    @Override
    public void init() {
        robot = new Robot(this);
        camera = new Camera(this);
        time = new ElapsedTime();

        oopState = new Wait();      // initialize the oopState object to Wait

        robot.telemetry.addData("Alliance Color", allianceColorIsRed ? "Red" : "Blue");
        robot.telemetry.addData("Skypos", skystone);
        robot.telemetry.update();

        robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
        robot.autoGrab.setGrabState(OPEN);
        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
        // set up bulk reads for motors - wrong
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module : allHubs) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
    }


    @Override
    public void init_loop() {
        updateWaitTime();       // increase / decrease wait time with GP1's dpad up and dpad down
        updateAllianceColor();  // flip allianceColor based on gamepad1.x
        updateStonesToPlace();
//        skystone = camera.getSkyPos(allianceColorIsRed);
        skystone = 1;
        currentStone = skystone;
        telemetry.addData("Wait time", waitTime);
        telemetry.addData("Alliance Color", allianceColorIsRed ? "Red" : "Blue");
        telemetry.addData("Stones to place", stonesToPlace);
        telemetry.update();
        robot.stickyGamepad1.update();

    }


    @Override
    public void start() {
        HEADING = allianceColorIsRed ? UP : DOWN;
        FINAL_BRIDGE_DISTANCE = allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE;
        FINAL_PICK_Y = allianceColorIsRed ? pickY : -pickY;
        OUT = (allianceColorIsRed ? RIGHT : LEFT);

        quarryStonePoses = (allianceColorIsRed ? redQuarryStonePoses : blueQuarryStonePoses);

        // shift the positions by a few inches each way to account for turning grab
        for (int i = 0; i < 6; i++) {
            quarryStonePoses[i][0] += TURN_GRAB_ADJUST;
        }

        if (allianceColorIsRed) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, UP));
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, 62, DOWN));
        }

        fillStonesArray();
    }


    @Override
    public void loop() {


        /**
         *   Auto flow
         *
         *   Wait
         *   Stone 1
         *       Wall to close Skystone
         *       Grab stone
         *       Stone to foundation
         *       Deposit block
         *   Stone 2
         *       Foundation to next Skystone
         *       Grab stone
         *       Stones to foundation
         *       Deposit block
         *   Stone 3
         *       Foundation to closest stone
         *       Grab stone
         *       Stones to foundation
         *       Deposit block
         *   Stone 4
         *       Foundation to closest stone
         *       Grab stone
         *       Stones to foundation
         *       Deposit block
         *   Stone 5
         *       Foundation to closest stone
         *       Grab stone
         *       Stones to foundation
         *       Deposit block
         *   Stone 6
         *       Foundation to closest stone
         *       Grab stone
         *       Stones to foundation
         *       Deposit block
         *       Grab foundation
         *       Pull back, turn, and push foundation into corner
         *       Scissor/tape park
         *   Idle
         *
         * */


        // OOP auto
        robot.mecanumDrive.updatePoseEstimate();
        currentPos = robot.mecanumDrive.getPoseEstimate();

        oopState = oopState.doLoop();

        robot.telemetry.addLine("Current State is " + oopState);
        robot.telemetry.addData("Current position", currentPos);
        robot.telemetry.addData("Grab state", robot.autoGrab.grabState);
        robot.telemetry.addData("Time", time);
        robot.telemetry.addData("Current Stone ", currentStone);
        robot.telemetry.addData("Stones Placed", stonesPlaced);
        robot.telemetry.update();

    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Abstract base AutoStates */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    // base class for all AutoState classes such as Wait, Grab
    private abstract class AutoState {

        /**
         * Does methods of subclasses
         *
         * @return what state to run next loop
         */
        public abstract AutoState doLoop();

    }

    // base class for states with a Trajectory
    private abstract class AutoStateWithTrajectory extends AutoState {

        /**
         * Gets the trajectory for the current state
         *
         * @return the trajectory to use in the follower
         */
        protected abstract Trajectory getTrajectory();

        /**
         * Gets the next state for oopState to become
         * Called once the robot arrives at the end of the Trajectory
         *
         * @return the next state
         */
        public abstract AutoState getNextState();

        /**
         * Starts the follower with the provided path from getTrajectory
         */
        protected void initState() {
            inited = true;
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
        }

        /**
         * Moves oopState to the next state if the robot has arrived at the end of the trajectory
         *
         * @return the AutoState for the next loop cycle
         */
        public AutoState doLoop() {
            if (!inited) {
                initState();
            }
            followTrajectory();
            if (hasArrived()) {
                time.reset();
                inited = false;
                return getNextState();
            } else {
                return this;
            }
        }

        boolean inited;     // used in doLoop() and init()

    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Concrete AutoStates */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    // wait a specified time before starting Auto
    private class Wait extends AutoState {

        @Override
        public AutoState doLoop() {

            if (time.seconds() < waitTime) {
                return this;
            } else {
                return WALL_TO_FIRST_BLOCK;
            }
        }
    }


    // path from wall to the skystone detected closest to the bridge
    private WallToFirstBlock WALL_TO_FIRST_BLOCK = new WallToFirstBlock();

    private class WallToFirstBlock extends AutoStateWithTrajectory {

        @Override
        protected Trajectory getTrajectory() {
            robot.autoGrab.setRotateState(AutoGrab.RotateState.MIDDLE);
            robot.autoGrab.setTurnState(allianceColorIsRed ? AutoGrab.TurnState.RIGHT : AutoGrab.TurnState.LEFT);
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(quarryStonePoses[skystone][0],
                                    FINAL_PICK_Y),
                            new ConstantInterpolator(HEADING))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            return ZERO_POSITION;
        }
    }


    // zero the position after the follower to center on the stone
    private ZeroPosition ZERO_POSITION = new ZeroPosition();

    private class ZeroPosition extends AutoState {
        boolean inited = false;

        public void setZeroPos() {
            robot.mecanumDrive.goToPosition(new Pose2d(
                    quarryStonePoses[currentStone][0] + pickXAdd,
                    FINAL_PICK_Y,
                    HEADING));
        }

        @Override
        public AutoState doLoop() {

            if (!inited) {
                setZeroPos();
                inited = true;
            }

            robot.mecanumDrive.updateGoToPos();
            if (robot.mecanumDrive.isInRange()) {
                robot.mecanumDrive.stopDriveMotors();
                inited = false;
                return GRAB;
            }
            return this;
        }
    }


    // grab the block in front of the robot (no xy movement)
    private Grab GRAB = new Grab();

    private class Grab extends AutoState {

        boolean inited = false;

        public void resetGrabTime() {
            time.reset();
            inited = true;
        }


        @Override
        public AutoState doLoop() {
            robot.mecanumDrive.stopDriveMotors();
            if (!inited) {
                resetGrabTime();
            }

            if (time.seconds() < GRAB_TIME + 0.05) {
                grabBlock();
                return this;
            } else {
                inited = false;
                return STONES_TO_FOUNDATION;
            }
        }

        private void grabBlock() {
            if (time.seconds() < GRAB_DIFF_TIME) {
                robot.autoGrab.setRotateState(AutoGrab.RotateState.DOWN);
                robot.autoGrab.setTurnState(allianceColorIsRed ? AutoGrab.TurnState.RIGHT : AutoGrab.TurnState.LEFT);
            } else if (time.seconds() < GRAB_TIME) {
                robot.autoGrab.setGrabState(AutoGrab.GrabState.GRAB);
            } else {
                robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
            }
        }
    }


    // path from stones to foundation by bridge
    public StonesToFoundation STONES_TO_FOUNDATION = new StonesToFoundation();

    private class StonesToFoundation extends AutoStateWithTrajectory {

        public Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(-12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(HEADING))
                    .lineTo(new Pose2d(0,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(HEADING))
                    .lineTo(new Pose2d(12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(HEADING))
                    .lineTo(new Pose2d(20,
                                    -34).vec(),
                            new ConstantInterpolator(HEADING))
                    .addMarker(() -> {
                        // turn straight for placing, lower arm to halfway for quicker place
                        robot.autoGrab.setRotateState(AutoGrab.RotateState.MIDDLEUP);
                        robot.autoGrab.setTurnState(AutoGrab.TurnState.MIDDLE);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(placeX, FINAL_BRIDGE_DISTANCE),
                            new ConstantInterpolator(HEADING))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            return PLACE_STONE;
        }
    }


    // place a stone on the foundation (no xy movement)
    private PlaceStone PLACE_STONE = new PlaceStone();

    private class PlaceStone extends AutoState {

        @Override
        public AutoState doLoop() {
            if (time.seconds() < PLACE_TIME + 0.05) {
                placeStone();
                return this;
            } else {
                stonesPlaced++;
                if (stonesPlaced < stonesToPlace) {
                    return FOUNDATION_TO_STONES;
                } else {
                    return TURN_AND_GRAB_FOUNDATION;
                }
            }
        }

        public void placeStone() {
            if (time.seconds() < PLACE_DIFF_TIME) {
                robot.autoGrab.setRotateState(AutoGrab.RotateState.DOWN);
            } else if (time.seconds() < PLACE_TIME) {
                robot.autoGrab.setGrabState(OPEN);
            } else {
                robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
            }
        }
    }


    // path from foundation to next stone
    private FoundationToStones FOUNDATION_TO_STONES = new FoundationToStones();

    private class FoundationToStones extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {

            robot.mecanumDrive.setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY() - 0.125, currentPos.getHeading()));
            //;)
            placeX -= 4;
            double BRIDGE_OFFSET = 0;//THIS IS A TEST VALUE SINCE IT IS HITTING SKYBRIDGE
            setNextStone();
            robot.autoGrab.setTurnState(allianceColorIsRed ? AutoGrab.TurnState.RIGHT : AutoGrab.TurnState.LEFT);
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(12,
                                    FINAL_BRIDGE_DISTANCE - BRIDGE_OFFSET),
                            new ConstantInterpolator(HEADING))
                    .lineTo(new Vector2d(0,
                                    FINAL_BRIDGE_DISTANCE - BRIDGE_OFFSET),
                            new ConstantInterpolator(HEADING))
                    .lineTo(new Vector2d(-12,
                                    FINAL_BRIDGE_DISTANCE - BRIDGE_OFFSET),
                            new ConstantInterpolator(HEADING))
                    .addMarker(() -> {
                        robot.autoGrab.setRotateState(AutoGrab.RotateState.MIDDLE);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(quarryStonePoses[getNextStone()][0] + pickXAdd,
                                    FINAL_PICK_Y),
                            new ConstantInterpolator(HEADING))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return ZERO_POSITION;
        }
    }

    private TurnAndGrabFoundation TURN_AND_GRAB_FOUNDATION = new TurnAndGrabFoundation();

    private class TurnAndGrabFoundation extends AutoState {
        boolean inited = false;
        double FOUNDATION_GRAB_TIME = 0.05;
        boolean rotated = false;

        public void setPos() {
            robot.mecanumDrive.goToPosition(new Pose2d(
                    48,
                    allianceColorIsRed ? -34 : 34,
                    OUT));
        }

        @Override
        public AutoState doLoop() {

            if (!inited) {
                setPos();
                inited = true;
            }
            if (!rotated) {
                robot.mecanumDrive.updateGoToPos();
                robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRABSET);
                if (robot.mecanumDrive.isInRange(1, 3)) {
                    rotated = true;
                    time.reset();
                }
            } else {
                if (time.seconds() < 0.4) {
                    robot.mecanumDrive.setMecanum(0, 0.3, 0);
                } else if (time.seconds() < 0.4 + FOUNDATION_GRAB_TIME) {
                    robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);

                } else {
                    robot.mecanumDrive.stopDriveMotors();
                    return MOVE_FOUNDATION_2;

                }
            }
            return this;
        }
    }

    // move the foundation from close to the bridge b`                      `   ````````````````````````````````````ack, turn, and push against wall
    private MoveFoundation2 MOVE_FOUNDATION_2 = new MoveFoundation2();

    private class MoveFoundation2 extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {

            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(28,
                            (allianceColorIsRed ? -40 : 40),
                            Math.toRadians(allianceColorIsRed ? 135 : 225)))
                    .reverse()
                    .splineTo(new Pose2d(52,
                            (allianceColorIsRed ? -48 : 48),
                            UP))

                    .build();
        }

        @Override
        public AutoState getNextState() {
            return PARK;
        }
    }


    // activate scissor park once the foundation is in place
    private Park PARK = new Park();

    private class Park extends AutoStateWithTrajectory {

        @Override
        protected Trajectory getTrajectory() {
            robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(0, allianceColorIsRed?-36:36), new ConstantInterpolator(DOWN))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return IDLE;
        }
    }


    // end phase once robot is parked - does nothing
    private Idle IDLE = new Idle();

    public class Idle extends AutoState {

        @Override
        public AutoState doLoop() {
            robot.mecanumDrive.stopDriveMotors();
            robot.mecanumDrive.setZeroPowerMode();
            return this;        // stay in the same state
        }
    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Utility Methods */
    /*--------------------------------------------------------------------------------------------------------------------------*/


    private boolean hasArrived() {
        return !robot.mecanumDrive.follower.isFollowing();
    }


    private void followTrajectory() {
        robot.mecanumDrive.updateFollowingDrive();
    }


    // increase or decrease waitTime based on dpad up and down
    public void updateWaitTime() {
        if (robot.stickyGamepad1.dpad_up == tempUp) {
            waitTime = Range.clip(waitTime + 0.5, 0, 30);
            tempUp = !tempUp;
        }
        if (robot.stickyGamepad1.dpad_down == tempDown) {
            waitTime = Range.clip(waitTime - 0.5, 0, 30);
            tempDown = !tempDown;
        }
    }


    // flip allianceColor based on gamepad1.x
    private void updateAllianceColor() {
        allianceColorIsRed = !robot.stickyGamepad1.x;
    }


    private void updateStonesToPlace() {
        if (robot.stickyGamepad1.dpad_left == tempLeft) {
            stonesToPlace = Range.clip(stonesToPlace - 1, 0, 6);
            tempLeft = !tempLeft;
        }
        if (robot.stickyGamepad1.dpad_right == tempRight) {
            stonesToPlace = Range.clip(stonesToPlace + 1, 0, 6);
            tempRight = !tempRight;
        }
    }


    // remove the stone that was just moved and make currentStone the next stone to get
    private void setNextStone() {
        stones.remove(0);
        currentStone = stones.get(0);
    }


    private int getNextStone() {
        return currentStone;
    }


    // fills the stones ArrayList in the order we want the robot to get the stones
    private void fillStonesArray() {
        stones.add(skystone);
        stones.add(skystone + 3);
        if (skystone == 0) {
            stones.add(1);
            stones.add(2);
            stones.add(4);
            stones.add(5);
        } else if (skystone == 1) {
            stones.add(0);
            stones.add(2);
            stones.add(3);
            stones.add(5);
        } else {
            stones.add(0);
            stones.add(1);
            stones.add(3);
            stones.add(4);
        }
    }

/**
 WAIT,                           // wait a specified time before starting Auto
 WALL_TO_FIRST_BLOCK,            // path from wall to the skystone detected closest to the bridge
 GRAB,                           // grab the block in front of the robot (no xy movement)
 FIRST_STONE_TO_FOUNDATION,      // path from the first stone to the foundation starting position
 MOVE_FOUNDATION_1,              // move the foundation from starting position to close to the bridge
 FOUNDATION_TO_STONES,           // path from foundation to next stone
 STONES_TO_FOUNDATION,           // path from stones to foundation by bridge
 PLACE_STONE,                    // place a stone on the foundation (no xy movement)
 MOVE_FOUNDATION_2,              // move the foundation from close to the bridge back, turn, and push against wall
 PARK,                           // activate scissor park once the foundation is in place
 IDLE                            // end phase once robot is parked - does nothing
 */
}
