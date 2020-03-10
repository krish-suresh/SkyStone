package org.firstinspires.ftc.teamcode.SkyStone.V3.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
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

@Autonomous(name = "HybridAuto")
public class HybridAuto extends Auto {

    static double IN;
    static double OUT;

    private double redPlaceY = -40;
    private double bluePlaceY = 40;
    private double placeY;
    private double placeAddY = 4;
    private double placeX2 = 36;        // for pushed foundation location

    public boolean switched;
    private double autoAddLiftPower = 0;

    private double intakePrepAngle;
    private double intakeAngle;

    private int bridgeOffset;

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

        intakePrepAngle = allianceColorIsRed ? 155 : 25;
        intakeAngle = allianceColorIsRed ? 170 : 10;

        bridgeOffset = allianceColorIsRed ? 2 : -2;
    }

    @Override
    public void loop() {
        if (stonesPlaced < 2) {
            super.loop();       // ends at the end of PlaceStone of the second stone from grab auto
        } else if (!switched) {
            super.FINAL_BRIDGE_DISTANCE -= allianceColorIsRed ? 3 : -3;
            robot.depositLift.setTargetHeight(0);
            oopState = new TurnAndGrabFoundation();     // TODO: Make this logic call the Auto's State instead of the one int
            switched = true;
        } else {
            robot.mecanumDrive.updatePoseEstimate();
            currentPos = robot.mecanumDrive.getPoseEstimate();
            robot.depositLift.updateLiftPower(robot.depositLift.pidAutonomous.update(robot.depositLift.getAbsLiftHeight()) + autoAddLiftPower);
            oopState = oopState.doLoop(time, robot);
            robot.telemetry.addLine("Current State is " + oopState.toString());
            robot.telemetry.addData("Current position", currentPos);
            robot.telemetry.addData("Time", time);
            robot.telemetry.addData("Current Stone ", currentStone);
            robot.telemetry.addData("Stones Placed", stonesPlaced);
            robot.telemetry.update();
        }
    }


    /*--------------------------------------------------------------------------------------------------------------------------*/
    /* Concrete AutoStates for Intake */
    /*--------------------------------------------------------------------------------------------------------------------------*/
//
    private class TurnAndGrabFoundation extends AutoState {
        boolean inited = false;
        double FOUNDATION_GRAB_TIME = 0.1;
        boolean rotated = false;

        public void setPos() {
            robot.mecanumDrive.goToPosition(new Pose2d(
                    currentPos.getX(),
                    currentPos.getY(),
                    OUT));
        }

        @Override
        public AutoState doLoop(ElapsedTime time, Robot robot) {

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
                    robot.autoGrab.setGrabState(AutoGrab.GrabState.GRAB);
                    robot.autoGrab.setRotateState(AutoGrab.RotateState.UP);
                    robot.autoGrab.setTurnState(AutoGrab.TurnState.FARRIGHT);
                    return MOVE_FOUNDATION;

                }
            }
            return this;
        }
    }


    // path from the first stone to the foundation starting position
    private MoveFoundation MOVE_FOUNDATION = new MoveFoundation();

    private class MoveFoundation extends AutoStateWithTrajectory {

        protected Trajectory getTrajectory() {

            /*.splineTo(new Pose2d(28,
                            (allianceColorIsRed ? -40 : 40),
                            Math.toRadians(allianceColorIsRed ? 135 : 225)))
                    .reverse()
                    .splineTo(new Pose2d(52,
                            (allianceColorIsRed ? -48 : 48),
                            UP))*/

            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())

                    .splineTo(new Pose2d(36, FINAL_BRIDGE_DISTANCE,
                            DOWN))
                    .addMarker(() -> {
                        // release the foundation
                        robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
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

    private class FoundationToStones extends AutoStateWithTrajectory { // TODO: switch all the paths to splines

        protected Trajectory getTrajectory() {
            autoAddLiftPower = 0;
            robot.mecanumDrive.setFoundationGrab(MecanumDriveBase.FoundationGrabState.RELEASED);
            robot.depositLift.releaseStone();
            robot.depositLift.setExtend(DepositLift.ExtendStates.TELE_GRAB);
            double intakeAddXVal = 0;
            switch (stonesPlaced) {
                case 2:
                    intakeAddXVal = 4;
                    break;
                case 3:
                    intakeAddXVal = -2;
                    break;
                default:
                    intakeAddXVal = 4;
                    break;
            }
            setNextStone();
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .splineTo(new Pose2d(24,
                                            FINAL_BRIDGE_DISTANCE,
                                            DOWN))
                    .addMarker(() -> {
                        // bring down the lift to clear the bar
                        robot.depositLift.setTargetHeight(0);
                        autoAddLiftPower = -0.5;
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(0,
                            FINAL_BRIDGE_DISTANCE,
                            DOWN))
                    .addMarker(() -> {
                        // get ready to intake block
                        // fix collector pos
                        robot.intake.setCollectorPos(Intake.CollectorPoses.MIDDLE);
                        // start spinning intake
                        robot.intake.setIntakePower(0.8);
                        robot.depositLift.setTargetHeight(1);
                        autoAddLiftPower = 0.2;
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(quarryStonePoses[currentStone][0] + intakeAddXVal,
                                allianceColorIsRed ? -22 : 22, Math.toRadians(intakePrepAngle)),
                                new SplineInterpolator(DOWN, Math.toRadians(intakePrepAngle)))
                    .addMarker(() -> {
                        // stop bringing the lift up
                        autoAddLiftPower = 0;
                        // close on the block once we reach where it should be
                        robot.intake.setCollectorPos(Intake.CollectorPoses.CLOSED);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(quarryStonePoses[currentStone][0] + intakeAddXVal,
                                allianceColorIsRed ? -22 : 22).vec(),
                                new SplineInterpolator(DOWN, Math.toRadians((intakeAngle))
                                /*currentPos.getHeading(), new ConstantInterpolator(Math.toRadians(intakeAngle)*/))
//                    .lineTo(new Vector2d(quarryStonePoses[currentStone][0]+16,
//                                    FINAL_BRIDGE_DISTANCE),
//                            new SplineInterpolator(DOWN,/*getHeadingToStone(quarryStonePoses[currentStone])*/Math.toRadians(135)))

//                    .lineTo(new Vector2d(quarryStonePoses[currentStone][0] +10,
//                                    quarryStonePoses[currentStone][1] + (allianceColorIsRed ? 2 : -2)),
//                            new ConstantInterpolator(/*getHeadingToStone(quarryStonePoses[currentStone])*/Math.toRadians(135)))
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

//            placeY += placeAddY; // TODO Taken out for testing

            FINAL_BRIDGE_DISTANCE -= bridgeOffset;

            stonesPlaced++;
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Pose2d(-12,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new SplineInterpolator(Math.toRadians(intakeAngle), DOWN))
                    .lineTo(new Pose2d(-6,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // bring the lift down
                        robot.depositLift.setTargetHeight(-2);
                        autoAddLiftPower = -0.3;
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(0,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // grab the block once the lift is down
                        robot.depositLift.grabStone();
                        robot.intake.setIntakePower(-1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(22,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // lift the lift once we clear the bridge
                        robot.depositLift.setTargetHeight(16);
                        robot.intake.setIntakePower(0);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Pose2d(30,
                                    FINAL_BRIDGE_DISTANCE).vec(),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // extend once lift is up
                        robot.depositLift.setExtend(DepositLift.ExtendStates.FULL_EXTEND);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(placeX2 - 2,
                                    placeY),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // release block
                        robot.depositLift.releaseStone();
                        // lift the lift another inch up
                        robot.depositLift.setTargetHeight(17);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(placeX2,
                                    placeY),
                            new ConstantInterpolator(DOWN))
                    .addMarker(() -> {
                        // bring lift back in
                        robot.depositLift.setExtend(DepositLift.ExtendStates.TELE_GRAB);
                        return Unit.INSTANCE;
                    })
                    .build();
        }

        @Override
        public AutoState getNextState() {
            time.reset();
            if (stonesPlaced < 4) {
                return FOUNDATION_TO_STONES;
            } else {
                return PARK;
            }
        }
    }


    // activate scissor park once the foundation is in place
    private Park PARK = new Park();

    private class Park extends AutoStateWithTrajectory {

        @Override
        protected Trajectory getTrajectory() {
            robot.depositLift.setTargetHeight(0);
            autoAddLiftPower = -0.5;
            return new TrajectoryBuilder(currentPos, robot.mecanumDrive.getConstraints())
                    .lineTo(new Vector2d(0, allianceColorIsRed ? -38 : 38), new ConstantInterpolator(DOWN))
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
        public AutoState doLoop(ElapsedTime time, Robot robot) {
            robot.mecanumDrive.stopDriveMotors();
            return this;        // stay in the same state
        }
    }


//    // TODO: FIX (DEFINITELY WRONG)
//    private double getHeadingToStone(double[] stonePos) {
//        // tangent of difference in Y over difference in X from currentPos to stonePos
//        return Math.tan((robot.mecanumDrive.getPoseEstimate().getY() - stonePos[1]) / (robot.mecanumDrive.getPoseEstimate().getX() - stonePos[0]));
//    }

}
