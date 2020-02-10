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

import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;

import kotlin.Unit;

@Autonomous (name = "Auto")
public class Auto extends OpMode {

    // Start object/value creation
    Robot robot;
    Camera camera;

    boolean tempUp = true;      // used for waitTime setting
    boolean tempDown = true;

    boolean movedFoundation = false;

    int skystone;       // from Camera, which position (0, 1, 2) the skystone is
    int stonesPlaced = 1;

    boolean allianceColorIsRed;

    double waitTime = 0;

    AutoState oopState;

    static final double UP = 0;
    static final double DOWN = Math.PI;
    Pose2d currentPos;

    final double[][] redQuarryStonePoses = {{-27.5, -22}, {-35.5, -22}, {-43.5, -22}, {-51.5, -22}, {-59.5, -22}, {-67.5, -22}};
    final double[][] blueQuarryStonePoses = {{-27.5, 22}, {-35.5, 22}, {-43.5, 22}, {-51.5, 22}, {-59.5, 22}, {-67.5, 22}};
    double[][] quarryStonePoses;

    ElapsedTime time;

    private double pickY = -36.5;                       // Y-distance at which we pick stones
    private double placeX = 48;                         // X-distance where we place stones on the foundation
    private double pickXAdd = 0;
    private double BRIDGE_DISTANCE = 44;
    private double FOUNDATION_PUSH_DISTANCE = 36;
    //End object/value creation



    @Override
    public void init() {
        robot = new Robot(this);
        camera = new Camera(this);
        time = new ElapsedTime();

        oopState = new Wait();      // initialize the oopState object to Wait

    }

    @Override
    public void init_loop() {
        updateWaitTime();       // increase / decrease wait time with GP1's dpad up and dpad down
        updateAllianceColor();  // flip allianceColor based on gamepad1.x
        skystone = camera.getSkyPos(allianceColorIsRed);        // TODO: fix skystone pipeline for 2 colors

    }

    @Override
    public void start() {
        quarryStonePoses = (allianceColorIsRed ? redQuarryStonePoses : blueQuarryStonePoses);

        if (allianceColorIsRed) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, UP));
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, DOWN));
        }

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
        *       Stone to foundation pos1
        *       Deposit block
        *       Grab foundation
        *       Move foundation closer to bridge
        *   Stone 2
        *       Foundation to next Skystone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 3
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 4
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block hehehehehe
        *   Stone 5
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *   Stone 6
        *       Foundation to closest stone
        *       Grab stone
        *       Stones to foundation pos2
        *       Deposit block
        *       Grab foundation
        *       Pull back, turn, and push foundation into corner
        *       Scissor park
        *   Idle
        *
        * */


        // OOP auto
        currentPos = robot.mecanumDrive.getPoseEstimate();

        oopState = oopState.doLoop();
        robot.telemetry.addLine("Current State is " + oopState.getStateName());
        robot.telemetry.update();

    }


/*--------------------------------------------------------------------------------------------------------------------------*/
    /* Abstract base AutoStates */
/*--------------------------------------------------------------------------------------------------------------------------*/


    // base class for all AutoState classes such as Wait, Grab
    private abstract class AutoState {

        /**
         * Does methods of subclasses
         * @return what state to run next loop
         */
        public abstract AutoState doLoop();

        /**
         * Used for telemetry of which state the robot is in
         * @return the name of the current state
         */
        public String getStateName() {
            return this.getClass().getName();
        }

    }

    // base class for states with a Trajectory
    private abstract class AutoStateWithTrajectory extends AutoState {

        /**
         * Gets the trajectory for the current state
         * @return the trajectory to use in the follower
         */
        protected abstract Trajectory getTrajectory();

        /**
         * Gets the next state for oopState to become
         * Called once the robot arrives at the end of the Trajectory
         * @return the next state
         */
        public abstract AutoState getNextState();

        /**
         * Starts the follower with the provided path from getTrajectory
         */
        protected void init() {
            inited = true;
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
            startFollowing();
        }

        /**
         * Moves oopState to the next state if the robot has arrived at the end of the trajectory
         * @return the AutoState for the next loop cycle
         */
        public AutoState doLoop() {
            if (!inited) {
                init();
            }
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
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(quarryStonePoses[skystone][0],
                                         allianceColorIsRed ? pickY : -pickY),
                                         new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return GRAB;
        }
    }


    // grab the block in front of the robot (no xy movement)
    private Grab GRAB = new Grab();
    private class Grab extends AutoState {

        private double grabTime = 0.4;

        @Override
        public AutoState doLoop() {
            if (time.seconds() < grabTime) {
                grabBlock();
                return this;
            } else {
                return (movedFoundation ? new StonesToFoundation() : new Stone1ToFoundation());
            }
        }

        private void grabBlock() {
            if (time.seconds() < 0.1) {
                robot.autoGrab.setGrabState(AutoGrab.GrabState.GRAB_DOWN);
            } else if (time.seconds() < 0.3) {
                robot.autoGrab.setGrabState(AutoGrab.GrabState.GRAB_UP);
            }
        }

    }


    // path from the first stone to the foundation starting position
    private class Stone1ToFoundation extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(-12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)).vec(),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Pose2d(0,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)).vec(),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Pose2d(12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)).vec(),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(placeX,
                                         allianceColorIsRed ? -29 : 29),
                                         new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(placeX,
                                         allianceColorIsRed ? -14.5 : 14.5),
                                         new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))       // add a lineTo just before the end
                    .addMarker(() -> {
                        //grab the platform once we reach where it should be
                        robot.autoGrab.setFoundationState(AutoGrab.FoundationState.DOWN);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(placeX, allianceColorIsRed ? -14 : 14), new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return MOVE_FOUNDATION_1;
        }
    }


    // move the foundation from starting position to close to the bridge
    private MoveFoundation1 MOVE_FOUNDATION_1 = new MoveFoundation1();
    private class MoveFoundation1 extends AutoStateWithTrajectory {



        protected Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(placeX - FOUNDATION_PUSH_DISTANCE,
                                            allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE),
                                            new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .addMarker(() -> {
                        //release the platform once we reach where it should be
                        robot.autoGrab.setFoundationState(AutoGrab.FoundationState.DOWN);
                        return Unit.INSTANCE;
                    })
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return FOUNDATION_TO_STONES;
        }
    }


    // path from foundation to next stone
    private FoundationToStones FOUNDATION_TO_STONES = new FoundationToStones();
    private class FoundationToStones extends AutoStateWithTrajectory {


        protected Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(0,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(-12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(quarryStonePoses[skystone][0] + pickXAdd,
                                            allianceColorIsRed ? pickY : -pickY),
                                            new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return GRAB;
        }
    }


    // path from stones to foundation by bridge
    private class StonesToFoundation extends AutoStateWithTrajectory {


        public Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(-12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(0,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(12,
                                          (allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE)),
                                          new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .lineTo(new Vector2d(placeX - FOUNDATION_PUSH_DISTANCE,
                                            allianceColorIsRed ? -31 : 31),
                                            new ConstantInterpolator(allianceColorIsRed ? UP : DOWN))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return PLACE_STONE;
        }
    }


    // place a stone on the foundation (no xy movement)
    private PlaceStone PLACE_STONE = new PlaceStone();
    private class PlaceStone extends AutoState {

        double placeTime = 0.1;

        @Override
        public AutoState doLoop() {
            if (time.seconds() < placeTime) {
                placeStone();
                return this;
            } else {
                stonesPlaced++;
                if (stonesPlaced < 6) {
                    return new FoundationToStones();
                } else {
                    return new MoveFoundation2();
                }
            }
        }

        public void placeStone() {
            robot.autoGrab.setGrabState(AutoGrab.GrabState.GRAB_DOWN);
        }

    }


    // move the foundation from close to the bridge back, turn, and push against wall
    private class MoveFoundation2 extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(28,
                                            (allianceColorIsRed ? -40 : 40),
                                            Math.toRadians(allianceColorIsRed ? 135 : 225))) // TODO
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
    private class Park extends AutoState {

        private double parkTime = 0.1;

        @Override
        public AutoState doLoop() {
            if (time.seconds() < parkTime) {
                // extend scissor lift
                return this;
            } else {
                return IDLE;
            }
        }

    }


    // end phase once robot is parked - does nothing
    private Idle IDLE = new Idle();
    public class Idle extends AutoState {

        @Override
        public AutoState doLoop() {
            return this;        // stay in the same state
        }
    }


/*--------------------------------------------------------------------------------------------------------------------------*/
    /* Utility Methods */
/*--------------------------------------------------------------------------------------------------------------------------*/


    private boolean hasArrived() {
        return !robot.mecanumDrive.follower.isFollowing();
    }

    private void startFollowing() {
        robot.mecanumDrive.updateFollowingDrive();
    }

    public void updateWaitTime() {
        if (robot.stickyGamepad1.dpad_up == tempUp) {
            waitTime += 0.5;
            tempUp = !tempUp;
        }
        if (robot.stickyGamepad1.dpad_down == tempDown) {
            waitTime -= 0.5;
            tempDown = !tempDown;
        }
    }

    // flip allianceColor based on gamepad1.x
    private void updateAllianceColor() {
        allianceColorIsRed = robot.stickyGamepad1.x;
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
