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
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.DepositLiftOld;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;


import java.util.ArrayList;

import kotlin.Unit;

@Autonomous (name = "AutoIntake")
public class IntakeAuto extends OpMode {

    /*
      Blue    |   Foundation side red
      (+,+)   |   (+,-)
              |
    --------------------
              |
      (-,+)   |   (-,-)
      Blue    |   Quarry side red
     */



    // Start object/value creation
    Robot robot;
    Camera camera;

    boolean tempUp = true;      // used for waitTime setting
    boolean tempDown = true;

    boolean movedFoundation = false;

    int skystone;       // from Camera, which position (0, 1, 2) the skystone is
    int stonesPlaced = 0;
    ArrayList<Integer> stones = new ArrayList<>(6);

    boolean allianceColorIsRed = true;

    double waitTime = 0;

    AutoState oopState;

    // directions are relative to the collector
    static final double UP = 0;
    static final double DOWN = Math.PI;
    static final double LEFT = Math.PI / 2;
    static final double RIGHT = -Math.PI / 2;
    static double IN;
    static double OUT;

    Pose2d currentPos;

    double[][] redQuarryStonePoses;
    double[][] blueQuarryStonePoses;
    double[][] quarryStonePoses;

    // TODO: test poses
    // predicted poses if the skystone is 0
    final double[][] redQuarryStonePoses0 = {{/*2*/-23.5, -22}, {/*3*/-34, -8}, {/*4*/-43, -8}, {/*1*/-47.5, -22}, {/*not used*/-55.5, -22}, {/*5*/-64, -8}};
    final double[][] blueQuarryStonePoses0 = {{-23.5, 22}, {-34, 8}, {-43, 8}, {-47.5, 22}, {/*not used*/-55.5, 22}, {-64, 8}};


    // predicted poses if the skystone is 1
    final double[][] redQuarryStonePoses1 = {{/*3*/-28.5, -4}, {/*2*/-31.5, -22}, {/*not used*/-39.5, -22}, {/*4*/-53.5, -5}, {/*1*/-55.5, -22}, {/*5*/-64, -8}};
    final double[][] blueQuarryStonePoses1 = {{-28.5, 4}, {-31.5, 22}, {/*not used*/-39.5, 22}, {-53.5, 5}, {-55.5, 22}, {-64, 8}};

    // predicted poses if the skystone is 2
    final double[][] redQuarryStonePoses2 = {{/*3*/-23.5, -22}, {/*4*/-34.5, -15}, {/*2*/-39.5, -22}, {/*not used*/-47.5, -22}, {/*5*/-57.5, -15}, {/*1*/-62, -22}};
    final double[][] blueQuarryStonePoses2 = {{-23.5, 22}, {-34.5, 15}, {-39.5, 22}, {/*not used*/-47.5, 22}, {-57.5, 15}, {-62, 22}};

    // outside = left for blue, right for red
    // pos0 based on top-outside corner, vertical
    // pos1 based on bottom-outside corner, horizontal
    // pos2 based on bottom-outside corner, horizontal
//    final double[][] redFoundationPoses = {{}, {}, {}};
//    final double[][] blueFoundationPoses = {{}, {}, {}};
//    double[][] foundationPoses;

    ElapsedTime time;

    private double redPlaceY = -40;
    private double bluePlaceY = 40;
    private double placeY;
    private double placeAddY;
    private double placeX1 = 48;        // for original foundation location
    private double placeX2 = 58;        // for pushed foundation location

    private double BRIDGE_DISTANCE = 44;
    private double FINAL_BRIDGE_DISTANCE;

    private int currentStone;


    //End object/value creation


    @Override
    public void init() {
        robot = new Robot(this);
        camera = new Camera(this);
        time = new ElapsedTime();

        oopState = new Wait();      // initialize the oopState object to Wait

        robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
        robot.autoGrab.setGrabState(AutoGrab.GrabState.OPEN);

        // set up bulk reads for motors - wrong
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module : allHubs) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
    }


    @Override
    public void init_loop() {
        updateWaitTime();       // increase / decrease wait time with GP1's dpad up and dpad down
        updateAllianceColor();  // flip allianceColor based on gamepad1.x  //TODO figure out why this doesn't work
        skystone = camera.getSkyPos(allianceColorIsRed);
        currentStone = skystone + 3;
        telemetry.addData("Wait time", waitTime);
        telemetry.addData("Alliance Color", allianceColorIsRed ? "Red" : "Blue");
        robot.telemetry.addData("Skypos", skystone);
        telemetry.update();

    }


    @Override
    public void start() {

        FINAL_BRIDGE_DISTANCE = allianceColorIsRed ? -BRIDGE_DISTANCE : BRIDGE_DISTANCE;

        redQuarryStonePoses = (skystone == 0 ?
                                    redQuarryStonePoses0 :
                                    (skystone == 1) ?
                                            redQuarryStonePoses1 :
                                            redQuarryStonePoses2);

        blueQuarryStonePoses = (skystone == 0 ?
                blueQuarryStonePoses0 :
                (skystone == 1) ?
                        blueQuarryStonePoses1 :
                        blueQuarryStonePoses2);

        quarryStonePoses = (allianceColorIsRed ? redQuarryStonePoses : blueQuarryStonePoses);
//        foundationPoses = (allianceColorIsRed ? redFoundationPoses : blueFoundationPoses);

        if (allianceColorIsRed) {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, -62, IN));
        } else {
            robot.mecanumDrive.setPoseEstimate(new Pose2d(-32.5, 62, IN));
        }

        placeY = (allianceColorIsRed ? redPlaceY : bluePlaceY);
        placeAddY = (allianceColorIsRed ? -4 : 4);

        IN = (allianceColorIsRed ? LEFT : RIGHT);
        OUT = (allianceColorIsRed ? RIGHT : LEFT);

        fillStonesArray();
    }


    @Override
    public void loop() {

        /**
         *      Auto flow
         *
         *      Wait
         *      Stone 1
         *          Forward 2 inches from wall
         *          Turn robot toward further skystone
         *          Path straight through skystone
         *          Close intake one inch into skystone
         *          Stone to foundation pos 0
         *          Place with deposit while grabbing foundation
         *          Rotate and pull foundation 20 inches from bridge
         *      Stone 2
         *          Foundation pos 1 to closer skystone
         *          Path through stone, closing grab
         *          Path back to foundation pos 1
         *          Deposit stone
         *          Push foundation to the back wall (pos 2)
         *      Stone 3
         *          Go for closest stone
         *              Pushed to the middle and facing almost horizontal, 20 inches from the midline
         *          Path through stone, closing grab at approximate location
         *          Path back to foundation pos 2
         *          Deposit stone
         *      Stone 4
         *          Go for one of the two middle stones
         *              Same path as Stone 3 but about 24 inches lower
         *          Path through stone, closing grab at approximate location
         *          Path to foundation pos 2
         *          Deposit
         *      Stone 5
         *          Go for stone pressed to the wall
         *          Path all the way down to the bottom wall, around 45 degree angle central of down
         *          Slide along the wall with active intake to a few inches past original stone line
         *          Close grab
         *          Path to foundation pos 2
         *          Deposit
         *      Park
         *      Idle
         *
         */


        // OOP auto
        robot.mecanumDrive.updatePoseEstimate();
        currentPos = robot.mecanumDrive.getPoseEstimate();

        oopState = oopState.doLoop();

        robot.telemetry.addLine("Current State is" + oopState);
        robot.telemetry.addData("Current position", currentPos);
        robot.telemetry.addData("Grab state", robot.autoGrab.grabState);
        robot.telemetry.addData("Time", time);
        robot.telemetry.addData("Current Stone", currentStone);
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
         * @return what state to run next loop
         */
        public abstract AutoState doLoop();

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
        protected void initState() {
            inited = true;
            robot.mecanumDrive.follower.followTrajectory(getTrajectory());
        }

        /**
         * Moves oopState to the next state if the robot has arrived at the end of the trajectory
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


    // path from wall to the skystone detected further from the bridge
    private WallToFirstBlock WALL_TO_FIRST_BLOCK = new WallToFirstBlock();
    private class WallToFirstBlock extends AutoStateWithTrajectory {

        @Override
        protected Trajectory getTrajectory() {

            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(currentPos.getX(),
                                    currentPos.getY() + (allianceColorIsRed ? 2 : -2)),
                            new ConstantInterpolator(currentPos.getHeading()))
                    .addMarker(() -> {
                        // get ready to intake block
                        // bring up lift
                        robot.depositLift.setTargetHeight(3);
                        // fix collector pos
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                        // start spinning intake
                        robot.intake.setIntakePower(0.7);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(quarryStonePoses[currentStone][0],
                                        quarryStonePoses[currentStone][1]),
                            new ConstantInterpolator(getHeadingToStone(quarryStonePoses[currentStone])))
                    .addMarker(() -> {
                        // close on the block once we reach where it should be
                        robot.intake.setCollectorPos(Intake.CollectorPoses.CLOSED);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(quarryStonePoses[currentStone][0] -2,
                                    quarryStonePoses[currentStone][1] + (allianceColorIsRed ? 2 : -2)),
                            new ConstantInterpolator(getHeadingToStone(quarryStonePoses[currentStone])))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            return new Stone1ToFoundation();
        }
    }

    // path from the first stone to the foundation starting position
    private class Stone1ToFoundation extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {
            stonesPlaced++;
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(-12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // bring down the lift and grab the block
                        robot.depositLift.setTargetHeight(3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(-8,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // bring down the lift and grab the block
                        robot.depositLift.grabStone();
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(6,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // lift the lift once we clear the bridge
                        robot.depositLift.setTargetHeight(6);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(7,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // extend once lift is up
                        robot.depositLift.setExtend(DepositLift.ExtendStates.STRAIGHT_PLACE);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .lineTo(new Vector2d(placeX1,
                                    allianceColorIsRed ? -31 : 31),
                            new ConstantInterpolator(OUT))      // with deposit/foundation grab facing foundation
                    .lineTo(new Vector2d(placeX1,
                                    allianceColorIsRed ? -29.5 : 29.5),
                            new ConstantInterpolator(OUT))       // add a lineTo just before the end
                    .addMarker(() -> {
                        // grab the platform once we reach where it should be
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.GRAB);
                        // bring down lift
                        robot.depositLift.setTargetHeight(1);
                        return Unit.INSTANCE;
                    })
                    .reverse()
                    .splineTo(new Pose2d(36,            // TODO: test
                            (allianceColorIsRed ? -48 : 48),
                            DOWN))
                    .addMarker(() -> {
                        // release block
                        robot.depositLift.releaseStone();
                        // bring lift back in
                        robot.depositLift.setExtend(DepositLift.ExtendStates.TELE_GRAB);
                        return Unit.INSTANCE;
                    })
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            return FOUNDATION_TO_STONES;
        }
    }


    // path from foundation to next stone
    private FoundationToStones FOUNDATION_TO_STONES = new FoundationToStones();
    private class FoundationToStones extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {
            placeY += 4;
            setNextStone();
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(12,
                                    FINAL_BRIDGE_DISTANCE),
                            new ConstantInterpolator(DOWN))
                    .lineTo(new Vector2d(0,
                                    FINAL_BRIDGE_DISTANCE),
                            new ConstantInterpolator(DOWN))
                    .lineTo(new Vector2d(-8,
                                    FINAL_BRIDGE_DISTANCE),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // get ready to intake block
                        // bring up lift
                        robot.depositLift.setTargetHeight(3);
                        // fix collector pos
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                        // start spinning intake
                        robot.intake.setIntakePower(0.7);
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(quarryStonePoses[currentStone][0],
                                    quarryStonePoses[currentStone][1]),
                            new ConstantInterpolator(getHeadingToStone(quarryStonePoses[currentStone])))
                    .addMarker(() -> {
                        // close on the block once we reach where it should be
                        robot.intake.setCollectorPos(Intake.CollectorPoses.CLOSED);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(quarryStonePoses[currentStone][0] -2,
                                    quarryStonePoses[currentStone][1] + (allianceColorIsRed ? 2 : -2)),
                            new ConstantInterpolator(getHeadingToStone(quarryStonePoses[currentStone])))
                    .build();
        }

        @Override
        public AutoState getNextState() {
            return STONES_TO_FOUNDATION;
        }
    }


//    // zero the position after the follower to center on the stone
//    private ZeroPosition ZERO_POSITION = new ZeroPosition();
//    private class ZeroPosition extends AutoState {
//        boolean inited = false;
//
//        public void setPos() {
//            robot.mecanumDrive.goToPosition(new Pose2d(
//                    quarryStonePoses[currentStone][0] + pickXAdd,
//                    FINAL_PICK_Y,
//                    HEADING));
//        }
//
//        @Override
//        public AutoState doLoop() {
//
//            if (!inited) {
//                setPos();
//                inited = true;
//            }
//
//            robot.mecanumDrive.updateGoToPos();
//            if (robot.mecanumDrive.isInRange()) {
//                inited = false;
//                return GRAB;
//            }
//            return this;
//        }
//    }


    // path from stones to foundation by bridge
    public StonesToFoundation STONES_TO_FOUNDATION = new StonesToFoundation();
    private class StonesToFoundation extends AutoStateWithTrajectory {

        public Trajectory getTrajectory() {
            placeY += placeAddY;
            stonesPlaced++;
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(-12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // bring down the lift
                        robot.depositLift.setTargetHeight(3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(-8,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // grab the block once the lift is down
                        robot.depositLift.grabStone();
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(6,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // lift the lift once we clear the bridge
                        robot.depositLift.setTargetHeight(6);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(7,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .addMarker(() -> {
                        // extend once lift is up
                        robot.depositLift.setExtend(DepositLift.ExtendStates.STRAIGHT_PLACE);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(UP))
                    .lineTo(new Vector2d(placeX2 - 2,
                                    placeY),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // bring the lift down
                        robot.depositLift.setTargetHeight(1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(placeX2,
                                    placeY),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // release block
                        robot.depositLift.releaseStone();
                        // bring lift back in
                        robot.depositLift.setExtend(DepositLift.ExtendStates.TELE_GRAB);
                        return Unit.INSTANCE;
                    })
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            if (stonesPlaced < 5) {
                return FOUNDATION_TO_STONES;
            } else {
                return PARK;
            }

        }
    }


//    // place a stone on the foundation (no xy movement)
//    private PlaceStone PLACE_STONE = new PlaceStone();
//    private class PlaceStone extends AutoState {
//
//        @Override
//        public AutoState doLoop() {
//            if (time.seconds() < PLACE_TIME + 0.05) {
//                placeStone();
//                return this;
//            } else {
//                stonesPlaced++;
//                if (stonesPlaced < 5) {
//                    return FOUNDATION_TO_STONES;
//                } else {
//                    return PARK;
//                }
//            }
//        }
//
//        public void placeStone() {
//            if (time.seconds() < GRAB_DIFF_TIME) {
//                robot.autoGrab.setGrabState(AutoGrab.GrabState.OPEN);
//            } else if (time.seconds() < PLACE_TIME) {
//                robot.autoGrab.setRotateState(AutoGrab.RotateState.DOWN);
//            } else {
//                robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
//            }
//        }
//    }


//    // move the foundation from close to the bridge back, turn, and push against wall
//    private MoveFoundation2 MOVE_FOUNDATION_2 = new MoveFoundation2();
//    private class MoveFoundation2 extends AutoStateWithTrajectory {
//
//        protected Trajectory getTrajectory() {
//            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
//                    .splineTo(new Pose2d(28,
//                            (allianceColorIsRed ? -40 : 40),
//                            Math.toRadians(allianceColorIsRed ? 135 : 225)))
//                    .reverse()
//                    .splineTo(new Pose2d(52,
//                            (allianceColorIsRed ? -48 : 48),
//                            UP))
//                    .build();
//        }
//
//        @Override
//        public AutoState getNextState() {
//            return PARK;
//        }
//    }


    // activate scissor park once the foundation is in place
    private Park PARK = new Park();
    private class Park extends AutoState {

        private double parkTime = 0.1;

        @Override
        public AutoState doLoop() {
            if (time.seconds() < parkTime) {
                // extend scissor lift / tape measurer
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


    public boolean hasArrived() {
        return !robot.mecanumDrive.follower.isFollowing();
    }


    public void followTrajectory() {
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
        allianceColorIsRed = robot.stickyGamepad1.x;
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
        stones.add(skystone + 3);
        stones.add(skystone);
        if (skystone == 0) {
            stones.add(1);
            stones.add(2);
            stones.add(5);
        } else if (skystone == 1) {
            stones.add(0);
            stones.add(3);
            stones.add(5);
        } else {
            stones.add(0);
            stones.add(1);
            stones.add(4);
        }
    }

    // TODO: FIX (DEFINITELY WRONG)
    private double getHeadingToStone(double[] stonePos) {
        // tangent of difference in Y over difference in X from currentPos to stonePos
        return Math.tan((robot.mecanumDrive.getPoseEstimate().getY() - stonePos[1]) / (robot.mecanumDrive.getPoseEstimate().getX() - stonePos[0]));
    }

/**
 WAIT,                           // wait a specified time before starting Auto
 WALL_TO_FIRST_BLOCK,            // path from wall to the skystone detected further to the bridge, intake stone
 FIRST_STONE_TO_FOUNDATION,      // path from the first stone to the foundation starting position, grab foundation, pull foundation while placing stone
 FOUNDATION_TO_STONES,           // path from foundation to next stone, intake stone
 STONES_TO_FOUNDATION,           // path from stones to foundation against the wall, place stone
 PARK,                           // activate scissor park once the foundation is in place
 IDLE                            // end phase once robot is parked - does nothing
 */

}
