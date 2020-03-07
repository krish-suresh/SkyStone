package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLibs.AutoState;
import org.firstinspires.ftc.teamcode.RobotLibs.AutoStateWithTrajectory;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.AutoGrab;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.SkyStone.V3.Subsystems.Robot;


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

@Autonomous(name = "BlendedAuto")
public class BlendedAuto extends Auto {


    static double IN;
    static double OUT;

    private double redPlaceY = -40;
    private double bluePlaceY = 40;
    private double placeY;
    private double placeAddY = 4;
    private double placeX2 = 58;        // for pushed foundation location

    public boolean switched;


    @Override
    public void init() {
        super.init();
    }


    public void init_loop() {
        super.init_loop();
    }


    public void start() {
        super.start();
        robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
        robot.autoGrab.setGrabState(AutoGrab.GrabState.OPEN);

        placeY = allianceColorIsRed ? redPlaceY : bluePlaceY;
        IN = allianceColorIsRed ? LEFT : RIGHT;
        OUT = allianceColorIsRed ? RIGHT : LEFT;
    }

    @Override
    public void loop() {
        if (stonesPlaced < 2) {
            super.loop();
        } else if (!switched) {
            oopState = new AStone1ToFoundation();
            switched = true;
        } else {
            oopState = oopState.doLoop(time, robot);
        }
    }


/*--------------------------------------------------------------------------------------------------------------------------*/
    /* Concrete AutoStates for Intake */
/*--------------------------------------------------------------------------------------------------------------------------*/


    // path from the first stone to the foundation starting position
    private class AStone1ToFoundation extends AutoStateWithTrajectory {

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
                    .lineTo(new Vector2d(placeX,
                                    allianceColorIsRed ? -31 : 31),
                            new ConstantInterpolator(OUT))      // with deposit/foundation grab facing foundation
                    .lineTo(new Vector2d(placeX,
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
            return PLACE_STONE;
        }
    }


    // place a stone on the foundation (no xy movement)
    private PlaceStone PLACE_STONE = new PlaceStone();
    private class PlaceStone extends AutoState {

        @Override
        public AutoState doLoop(ElapsedTime time, Robot robot) {
            if (time.seconds() < 0.85) {
                placeStone();
                return this;
            } else {
                stonesPlaced++;
                if (stonesPlaced < 5) {
                    return FOUNDATION_TO_STONES;
                } else {
                    return MOVE_FOUNDATION_2;
                }
            }
        }

        public void placeStone() {
            if (time.seconds() < 0.3) {
                robot.depositLift.setExtend(DepositLift.ExtendStates.STRAIGHT_PLACE);
            } else if (time.seconds() < 0.8) {
                robot.mecanumDrive.stopDriveMotors();
                robot.depositLift.releaseStone();
            } else {
                robot.depositLift.setExtend(DepositLift.ExtendStates.TELE_GRAB);
                robot.intake.setIntakePower(0);
            }
        }
    }


    // move the foundation from close to the bridge back, turn, and push against wall
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
    private class Park extends AutoState {

        private double parkTime = 0.1;

        @Override
        public AutoState doLoop(ElapsedTime time, Robot robot) {
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
        public AutoState doLoop(ElapsedTime time, Robot robot) {
            return this;        // stay in the same state
        }
    }


    // TODO: FIX (DEFINITELY WRONG)
    private double getHeadingToStone(double[] stonePos) {
        // tangent of difference in Y over difference in X from currentPos to stonePos
        return Math.tan((robot.mecanumDrive.getPoseEstimate().getY() - stonePos[1]) / (robot.mecanumDrive.getPoseEstimate().getX() - stonePos[0]));
    }


}
